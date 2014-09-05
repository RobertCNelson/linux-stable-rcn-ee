Vivante "clean" v4 driver with Flattened Device Tree support
------------------------------------------------------------

This version of the driver started from the "cleaned" v4 driver from
the GCW tree (https://github.com/gcwnow/linux). This version is targeted
for Linux kernel version 3.16 and onward.

This driver **DOES NOT BUILD** as a module. There are references to
functions that the 3.16 and onward kernels **DO NOT** export. This driver
only builds as an internal driver.

This driver supports OpenFirmware/Flattened Device Tree. If CONFIG_OF is set
in the kernel configuration, OpenFirmware/FDT support is enabled (this is
normally true for embedded systems.) The compatible names are:

	vivante,gpu3d
	galcore

An example from the Freescale i.MX6 DTS source:

		gpu3d: gpu@00130000 {
			compatible = "fsl,imx6qdl-gpu3d", "vivante,gpu3d", "galcore";
			reg = <0x00130000 0x4000>;
			interrupts = <0 9 IRQ_TYPE_LEVEL_HIGH>;
			interrupt-names = "galcore-gpu3d";
			clocks = <&clks IMX6QDL_CLK_GPU3D_CORE>;
		};

The 2D core can be optionally present, with the compatible name
"vivante,gpu2d". Again, from the Freescale i.MX6 DTS source:

		gpu2d: gpu@00134000 {
			compatible = "fsl,imx6qdl-gpu2d", "vivante,gpu2d";
			reg = <0x00134000 0x4000>;
			interrupts = <0 10 IRQ_TYPE_LEVEL_HIGH>,	/* R2D GPU2D general IRQ */
				     <0 11 IRQ_TYPE_LEVEL_HIGH>;	/* V2D GPU2D general IRQ */
			interrupt-names = "gpu2d", "gpuvd";
			clocks = <&clks IMX6QDL_CLK_GPU2D_CORE>;
		};

(Note: The 2D vector accelerator is not supported, despite being specified.
This is a FIXME for a later version.)

This driver also supports separate 3D and 2D clocks because the the Freescale
i.MX6 has separate clocks for the 3D and the 2D cores.
