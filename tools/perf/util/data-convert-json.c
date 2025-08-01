// SPDX-License-Identifier: GPL-2.0-only
/*
 * JSON export.
 *
 * Copyright (C) 2021, CodeWeavers Inc. <nfraser@codeweavers.com>
 */

#include "data-convert.h"

#include <fcntl.h>
#include <inttypes.h>
#include <sys/stat.h>
#include <unistd.h>

#include "linux/compiler.h"
#include "linux/err.h"
#include "util/auxtrace.h"
#include "util/debug.h"
#include "util/dso.h"
#include "util/event.h"
#include "util/evsel.h"
#include "util/evlist.h"
#include "util/header.h"
#include "util/map.h"
#include "util/session.h"
#include "util/symbol.h"
#include "util/thread.h"
#include "util/tool.h"

#ifdef HAVE_LIBTRACEEVENT
#include <event-parse.h>
#endif

struct convert_json {
	struct perf_tool tool;
	FILE *out;
	bool first;
	u64 events_count;
};

// Outputs a JSON-encoded string surrounded by quotes with characters escaped.
static void output_json_string(FILE *out, const char *s)
{
	fputc('"', out);
	while (*s) {
		switch (*s) {

		// required escapes with special forms as per RFC 8259
		case '"':  fputs("\\\"", out); break;
		case '\\': fputs("\\\\", out); break;
		case '\b': fputs("\\b", out);  break;
		case '\f': fputs("\\f", out);  break;
		case '\n': fputs("\\n", out);  break;
		case '\r': fputs("\\r", out);  break;
		case '\t': fputs("\\t", out);  break;

		default:
			// all other control characters must be escaped by hex code
			if (*s <= 0x1f)
				fprintf(out, "\\u%04x", *s);
			else
				fputc(*s, out);
			break;
		}

		++s;
	}
	fputc('"', out);
}

// Outputs an optional comma, newline and indentation to delimit a new value
// from the previous one in a JSON object or array.
static void output_json_delimiters(FILE *out, bool comma, int depth)
{
	int i;

	if (comma)
		fputc(',', out);
	fputc('\n', out);
	for (i = 0; i < depth; ++i)
		fputc('\t', out);
}

// Outputs a printf format string (with delimiter) as a JSON value.
__printf(4, 5)
static void output_json_format(FILE *out, bool comma, int depth, const char *format, ...)
{
	va_list args;

	output_json_delimiters(out, comma, depth);
	va_start(args, format);
	vfprintf(out,  format, args);
	va_end(args);
}

// Outputs a JSON key-value pair where the value is a string.
static void output_json_key_string(FILE *out, bool comma, int depth,
		const char *key, const char *value)
{
	output_json_delimiters(out, comma, depth);
	output_json_string(out, key);
	fputs(": ", out);
	output_json_string(out, value);
}

// Outputs a JSON key-value pair where the value is a printf format string.
__printf(5, 6)
static void output_json_key_format(FILE *out, bool comma, int depth,
		const char *key, const char *format, ...)
{
	va_list args;

	output_json_delimiters(out, comma, depth);
	output_json_string(out, key);
	fputs(": ", out);
	va_start(args, format);
	vfprintf(out,  format, args);
	va_end(args);
}

static void output_sample_callchain_entry(const struct perf_tool *tool,
		u64 ip, struct addr_location *al)
{
	struct convert_json *c = container_of(tool, struct convert_json, tool);
	FILE *out = c->out;

	output_json_format(out, false, 4, "{");
	output_json_key_format(out, false, 5, "ip", "\"0x%" PRIx64 "\"", ip);

	if (al && al->sym && al->sym->namelen) {
		struct dso *dso = al->map ? map__dso(al->map) : NULL;

		fputc(',', out);
		output_json_key_string(out, false, 5, "symbol", al->sym->name);

		if (dso) {
			const char *dso_name = dso__short_name(dso);

			if (dso_name && strlen(dso_name) > 0) {
				fputc(',', out);
				output_json_key_string(out, false, 5, "dso", dso_name);
			}
		}
	}

	output_json_format(out, false, 4, "}");
}

static int process_sample_event(const struct perf_tool *tool,
				union perf_event *event __maybe_unused,
				struct perf_sample *sample,
				struct evsel *evsel __maybe_unused,
				struct machine *machine)
{
	struct convert_json *c = container_of(tool, struct convert_json, tool);
	FILE *out = c->out;
	struct addr_location al;
	u64 sample_type = __evlist__combined_sample_type(evsel->evlist);
	u8 cpumode = PERF_RECORD_MISC_USER;

	addr_location__init(&al);
	if (machine__resolve(machine, &al, sample) < 0) {
		pr_err("Sample resolution failed!\n");
		addr_location__exit(&al);
		return -1;
	}

	++c->events_count;

	if (c->first)
		c->first = false;
	else
		fputc(',', out);
	output_json_format(out, false, 2, "{");

	output_json_key_format(out, false, 3, "timestamp", "%" PRIi64, sample->time);
	output_json_key_format(out, true, 3, "pid", "%i", thread__pid(al.thread));
	output_json_key_format(out, true, 3, "tid", "%i", thread__tid(al.thread));

	if ((sample_type & PERF_SAMPLE_CPU))
		output_json_key_format(out, true, 3, "cpu", "%i", sample->cpu);
	else if (thread__cpu(al.thread) >= 0)
		output_json_key_format(out, true, 3, "cpu", "%i", thread__cpu(al.thread));

	output_json_key_string(out, true, 3, "comm", thread__comm_str(al.thread));

	output_json_key_format(out, true, 3, "callchain", "[");
	if (sample->callchain) {
		unsigned int i;
		bool ok;
		bool first_callchain = true;

		for (i = 0; i < sample->callchain->nr; ++i) {
			u64 ip = sample->callchain->ips[i];
			struct addr_location tal;

			if (ip >= PERF_CONTEXT_MAX) {
				switch (ip) {
				case PERF_CONTEXT_HV:
					cpumode = PERF_RECORD_MISC_HYPERVISOR;
					break;
				case PERF_CONTEXT_KERNEL:
					cpumode = PERF_RECORD_MISC_KERNEL;
					break;
				case PERF_CONTEXT_USER:
					cpumode = PERF_RECORD_MISC_USER;
					break;
				default:
					pr_debug("invalid callchain context: %"
							PRId64 "\n", (s64) ip);
					break;
				}
				continue;
			}

			if (first_callchain)
				first_callchain = false;
			else
				fputc(',', out);

			addr_location__init(&tal);
			ok = thread__find_symbol(al.thread, cpumode, ip, &tal);
			output_sample_callchain_entry(tool, ip, ok ? &tal : NULL);
			addr_location__exit(&tal);
		}
	} else {
		output_sample_callchain_entry(tool, sample->ip, &al);
	}
	output_json_format(out, false, 3, "]");

#ifdef HAVE_LIBTRACEEVENT
	if (sample->raw_data) {
		struct tep_event *tp_format = evsel__tp_format(evsel);
		struct tep_format_field **fields = tp_format ? tep_event_fields(tp_format) : NULL;

		if (fields) {
			int i = 0;

			while (fields[i]) {
				struct trace_seq s;

				trace_seq_init(&s);
				tep_print_field(&s, sample->raw_data, fields[i]);
				output_json_key_string(out, true, 3, fields[i]->name, s.buffer);

				i++;
			}
			free(fields);
		}
	}
#endif
	output_json_format(out, false, 2, "}");
	addr_location__exit(&al);
	return 0;
}

static void output_headers(struct perf_session *session, struct convert_json *c)
{
	struct stat st;
	const struct perf_header *header = &session->header;
	const struct perf_env *env = perf_session__env(session);
	int ret;
	int fd = perf_data__fd(session->data);
	int i;
	FILE *out = c->out;

	output_json_key_format(out, false, 2, "header-version", "%u", header->version);

	ret = fstat(fd, &st);
	if (ret >= 0) {
		time_t stctime = st.st_mtime;
		char buf[256];

		strftime(buf, sizeof(buf), "%FT%TZ", gmtime(&stctime));
		output_json_key_string(out, true, 2, "captured-on", buf);
	} else {
		pr_debug("Failed to get mtime of source file, not writing captured-on");
	}

	output_json_key_format(out, true, 2, "data-offset", "%" PRIu64, header->data_offset);
	output_json_key_format(out, true, 2, "data-size", "%" PRIu64, header->data_size);
	output_json_key_format(out, true, 2, "feat-offset", "%" PRIu64, header->feat_offset);

	output_json_key_string(out, true, 2, "hostname", env->hostname);
	output_json_key_string(out, true, 2, "os-release", env->os_release);
	output_json_key_string(out, true, 2, "arch", env->arch);

	if (env->cpu_desc)
		output_json_key_string(out, true, 2, "cpu-desc", env->cpu_desc);

	output_json_key_string(out, true, 2, "cpuid", env->cpuid);
	output_json_key_format(out, true, 2, "nrcpus-online", "%u", env->nr_cpus_online);
	output_json_key_format(out, true, 2, "nrcpus-avail", "%u", env->nr_cpus_avail);

	if (env->clock.enabled) {
		output_json_key_format(out, true, 2, "clockid",
				"%u", env->clock.clockid);
		output_json_key_format(out, true, 2, "clock-time",
				"%" PRIu64, env->clock.clockid_ns);
		output_json_key_format(out, true, 2, "real-time",
				"%" PRIu64, env->clock.tod_ns);
	}

	output_json_key_string(out, true, 2, "perf-version", env->version);

	output_json_key_format(out, true, 2, "cmdline", "[");
	for (i = 0; i < env->nr_cmdline; i++) {
		output_json_delimiters(out, i != 0, 3);
		output_json_string(c->out, env->cmdline_argv[i]);
	}
	output_json_format(out, false, 2, "]");
}

int bt_convert__perf2json(const char *input_name, const char *output_name,
		struct perf_data_convert_opts *opts __maybe_unused)
{
	struct perf_session *session;
	int fd;
	int ret = -1;
	struct convert_json c = {
		.first = true,
		.events_count = 0,
	};
	struct perf_data data = {
		.mode = PERF_DATA_MODE_READ,
		.path = input_name,
		.force = opts->force,
	};

	perf_tool__init(&c.tool, /*ordered_events=*/true);
	c.tool.sample         = process_sample_event;
	c.tool.mmap           = perf_event__process_mmap;
	c.tool.mmap2          = perf_event__process_mmap2;
	c.tool.comm           = perf_event__process_comm;
	c.tool.namespaces     = perf_event__process_namespaces;
	c.tool.cgroup         = perf_event__process_cgroup;
	c.tool.exit           = perf_event__process_exit;
	c.tool.fork           = perf_event__process_fork;
	c.tool.lost           = perf_event__process_lost;
#ifdef HAVE_LIBTRACEEVENT
	c.tool.tracing_data   = perf_event__process_tracing_data;
#endif
	c.tool.build_id       = perf_event__process_build_id;
	c.tool.id_index       = perf_event__process_id_index;
	c.tool.auxtrace_info  = perf_event__process_auxtrace_info;
	c.tool.auxtrace       = perf_event__process_auxtrace;
	c.tool.event_update   = perf_event__process_event_update;
	c.tool.ordering_requires_timestamps = true;

	if (opts->all) {
		pr_err("--all is currently unsupported for JSON output.\n");
		goto err;
	}
	if (opts->tod) {
		pr_err("--tod is currently unsupported for JSON output.\n");
		goto err;
	}

	fd = open(output_name, O_CREAT | O_WRONLY | (opts->force ? O_TRUNC : O_EXCL), 0666);
	if (fd == -1) {
		if (errno == EEXIST)
			pr_err("Output file exists. Use --force to overwrite it.\n");
		else
			pr_err("Error opening output file!\n");
		goto err;
	}

	c.out = fdopen(fd, "w");
	if (!c.out) {
		fprintf(stderr, "Error opening output file!\n");
		close(fd);
		goto err;
	}

	session = perf_session__new(&data, &c.tool);
	if (IS_ERR(session)) {
		fprintf(stderr, "Error creating perf session!\n");
		goto err_fclose;
	}
	if (symbol__init(perf_session__env(session)) < 0) {
		fprintf(stderr, "Symbol init error!\n");
		goto err_session_delete;
	}

	// The opening brace is printed manually because it isn't delimited from a
	// previous value (i.e. we don't want a leading newline)
	fputc('{', c.out);

	// Version number for future-proofing. Most additions should be able to be
	// done in a backwards-compatible way so this should only need to be bumped
	// if some major breaking change must be made.
	output_json_format(c.out, false, 1, "\"linux-perf-json-version\": 1");

	// Output headers
	output_json_format(c.out, true, 1, "\"headers\": {");
	output_headers(session, &c);
	output_json_format(c.out, false, 1, "}");

	// Output samples
	output_json_format(c.out, true, 1, "\"samples\": [");
	perf_session__process_events(session);
	output_json_format(c.out, false, 1, "]");
	output_json_format(c.out, false, 0, "}");
	fputc('\n', c.out);

	fprintf(stderr,
			"[ perf data convert: Converted '%s' into JSON data '%s' ]\n",
			data.path, output_name);

	fprintf(stderr,
			"[ perf data convert: Converted and wrote %.3f MB (%" PRIu64 " samples) ]\n",
			(ftell(c.out)) / 1024.0 / 1024.0, c.events_count);

	ret = 0;
err_session_delete:
	perf_session__delete(session);
err_fclose:
	fclose(c.out);
err:
	return ret;
}
