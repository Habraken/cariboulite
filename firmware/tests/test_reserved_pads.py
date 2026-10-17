#!/usr/bin/env python3
"""Check actual top-level pad isolation with the installed iCE40 simulation model.

This exercises digital direction/selection behavior, not board electrical limits
or timing. It never synthesizes, programs hardware, or writes firmware artifacts.
"""
from pathlib import Path
import hashlib
import os
import shlex
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
BENCH = r"""`timescale 1ns/1ps
module tb;
    reg clock=0, lvds_clock=0, reset_n=0;
    always #4 clock=~clock;
    always #7.8125 lvds_clock=~lvds_clock;
    reg mosi=0, sck=0, select_n=1;
    reg [3:0] pmod_inputs=0;
    tri [6:0] reserved;
    reg [6:0] external_enable=0, external_value=0;
    tri miso;
    reg miso_external_enable=0, miso_external_value=0;
    tri [7:0] smi_data;
    wire request;
    integer checks=0;
    genvar pin;
    generate for(pin=0;pin<7;pin=pin+1) begin: external_drivers
        assign reserved[pin]=external_enable[pin] ? external_value[pin] : 1'bz;
    end endgenerate
    assign miso=miso_external_enable ? miso_external_value : 1'bz;

    top dut(
        .i_glob_clock(clock), .i_rst_b(reset_n),
        .o_rx_h_tx_l(), .o_rx_h_tx_l_b(), .o_tr_vc1(), .o_tr_vc1_b(),
        .o_tr_vc2(), .o_shdn_rx_lna(), .o_shdn_tx_lna(),
        .o_iq_tx_p(), .o_iq_tx_n(), .o_iq_tx_clk_p(), .o_iq_tx_clk_n(),
        .i_iq_rx_09_p(1'b0), .i_iq_rx_24_n(1'b0),
        .i_iq_rx_clk_p(lvds_clock),
        .o_mixer_fm(reserved[4]), .o_mixer_en(reserved[5]),
        .i_config(4'b0), .i_button(1'b1),
        .io_pmod_out(reserved[3:0]), .io_pmod_in(pmod_inputs),
        .o_led0(), .o_led1(), .i_smi_a2(1'b0), .i_smi_a3(1'b0),
        .i_smi_soe_se(1'b1), .i_smi_swe_srw(1'b1), .io_smi_data(smi_data),
        .o_smi_write_req(reserved[6]), .o_smi_read_req(request),
        .i_mosi(mosi), .i_sck(sck), .i_ss(select_n), .o_miso(miso)
    );

    // The standard SB_IO model does not model the PULLUP parameter electrically;
    // check the elaborated primitive parameters as well as observed pin behavior.
    generate for(pin=0;pin<4;pin=pin+1) begin: check_pmod_parameters
        initial begin
            if(dut.reserved_pmod_pad[pin].pad.PIN_TYPE !== 6'b000001 ||
               dut.reserved_pmod_pad[pin].pad.PULLUP !== 1'b0)
                $fatal(1,"PMOD reserved pad %0d is not input-only/no-pullup",pin);
        end
    end endgenerate

    task check_reserved;
        integer pattern;
        begin
            external_enable=0;
            #32;
            if(reserved !== 7'bzzzzzzz)
                $fatal(1,"reserved pads drive when externally released: %b",reserved);
            for(pattern=0;pattern<4;pattern=pattern+1) begin
                external_enable=7'h7f;
                case(pattern)
                    0: external_value=7'h00;
                    1: external_value=7'h7f;
                    2: external_value=7'h55;
                    3: external_value=7'h2a;
                endcase
                #32;
                if(reserved !== external_value)
                    $fatal(1,"reserved pads conflict with external driver: wanted %b got %b",
                           external_value,reserved);
            end
            external_enable=0;
            #32;
            if(reserved !== 7'bzzzzzzz)
                $fatal(1,"reserved pads did not return to high impedance");
            checks=checks+1;
        end
    endtask

    task spi_byte(input [7:0] value);
        integer bit_number;
        begin
            for(bit_number=7;bit_number>=0;bit_number=bit_number-1) begin
                mosi=value[bit_number];
                #100; sck=1;
                #100; sck=0;
            end
        end
    endtask

    task spi_write(input [7:0] command,input [7:0] value);
        begin
            select_n=1; sck=0;
            // The actual SPI receiver resets its bit count on a deselected edge.
            #100; sck=1;
            #100; sck=0;
            #100; select_n=0;
            spi_byte(command);
            #200;
            spi_byte(value);
            #400; select_n=1;
            #200;
            if(miso !== 1'bz) $fatal(1,"MISO drives after SPI deselection");
            check_reserved;
        end
    endtask

    integer channel,direction,full,empty;
    initial begin
        if(dut.reserved_mixer_fm_pad.PIN_TYPE !== 6'b000001 ||
           dut.reserved_mixer_fm_pad.PULLUP !== 1'b0 ||
           dut.reserved_mixer_en_pad.PIN_TYPE !== 6'b000001 ||
           dut.reserved_mixer_en_pad.PULLUP !== 1'b0 ||
           dut.reserved_smi_write_req_pad.PIN_TYPE !== 6'b000001 ||
           dut.reserved_smi_write_req_pad.PULLUP !== 1'b0)
            $fatal(1,"reserved mixer/SMI pad is not input-only/no-pullup");
        #80;
        check_reserved;
        reset_n=1;
        #160;
        check_reserved;

        // Check MISO's actual top-level output-enable path independently of
        // serializer contents, including another SPI device driving the bus.
        force dut.int_miso=1'b0;
        select_n=0; #16;
        if(miso !== 1'b0) $fatal(1,"selected MISO cannot drive zero");
        force dut.int_miso=1'b1;
        #16;
        if(miso !== 1'b1) $fatal(1,"selected MISO cannot drive one");
        select_n=1; #16;
        if(miso !== 1'bz) $fatal(1,"deselected MISO is not high impedance");
        miso_external_enable=1; miso_external_value=0; #16;
        if(miso !== 1'b0) $fatal(1,"MISO conflicts with another device's zero");
        miso_external_value=1; #16;
        if(miso !== 1'b1) $fatal(1,"MISO conflicts with another device's one");
        miso_external_enable=0;
        release dut.int_miso;

        // Actual SPI control writes must not enable any reserved output pad.
        spi_write(8'ha3,8'hff); // IO_CTRL PMOD direction storage
        if(dut.io_ctrl_ins.pmod_dir_state !== 8'hff)
            $fatal(1,"SPI PMOD direction write did not reach IO_CTRL");
        spi_write(8'ha4,8'h0f); // IO_CTRL PMOD value storage
        if(dut.io_ctrl_ins.pmod_state !== 4'hf)
            $fatal(1,"SPI PMOD value write did not reach IO_CTRL");
        spi_write(8'ha5,8'hff); // IO_CTRL RF-pin state storage
        spi_write(8'ha1,8'h01); // IO_CTRL direct RF debug mode
        spi_write(8'h86,8'hff); // SYS_CTRL sync types and TX sample gap
        if(dut.sys_ctrl_ins.tx_sample_gap !== 4'hf)
            $fatal(1,"SPI SYS_CTRL write did not reach sample-gap register");
        spi_write(8'h87,8'h0f); // SYS_CTRL software sync levels
        pmod_inputs=4'hf;
        check_reserved;

        // Force only FIFO boundary conditions; exercise the real request logic,
        // SPI channel/direction controls and top-level shared request pin.
        for(channel=0;channel<2;channel=channel+1) begin
            spi_write(8'hc2,channel);
            if(dut.channel !== channel[0]) $fatal(1,"SPI channel selection failed");
            for(direction=0;direction<2;direction=direction+1) begin
                spi_write(8'hc3,direction);
                if(dut.w_smi_data_direction !== direction[0])
                    $fatal(1,"SPI direction selection failed");
                for(full=0;full<2;full=full+1)
                for(empty=0;empty<2;empty=empty+1) begin
                    if(full) force dut.w_tx_fifo_full=1'b1;
                    else force dut.w_tx_fifo_full=1'b0;
                    if(empty) force dut.w_rx_fifo_empty=1'b1;
                    else force dut.w_rx_fifo_empty=1'b0;
                    #32;
                    if(request !== (direction ? !empty[0] : !full[0]))
                        $fatal(1,"SMI request changed: dir=%0d full=%0d empty=%0d req=%b",
                               direction,full,empty,request);
                    check_reserved;
                end
            end
        end
        release dut.w_tx_fifo_full;
        release dut.w_rx_fifo_empty;
        reset_n=0; #80;
        check_reserved;
        reset_n=1; #160;
        check_reserved;
        $display("PASS: seven reserved pads remain input-only/high-Z across reset, SPI writes and both channels/directions; external drivers do not conflict; MISO selection and SMI backpressure preserved (%0d pad checks)",checks);
        $finish;
    end
    initial begin
        #1000000;
        $fatal(1,"reserved-pad simulation timed out");
    end
endmodule
"""


def run(command):
    print("$ " + shlex.join(map(str, command)), flush=True)
    subprocess.run(command, cwd=ROOT, check=True, timeout=30)


data_roots = []
if os.environ.get("YOSYS_DATDIR"):
    data_roots.append(Path(os.environ["YOSYS_DATDIR"]))
data_roots.extend((Path("/usr/local/share/yosys"), Path("/usr/share/yosys")))
cells = next((directory / "ice40/cells_sim.v" for directory in data_roots
              if (directory / "ice40/cells_sim.v").is_file()), None)
if cells is None:
    raise SystemExit("iCE40 cells_sim.v is required; set YOSYS_DATDIR to the installed Yosys data directory")
print("iCE40 simulation model: " + str(cells), flush=True)
print("Model SHA-256: " + hashlib.sha256(cells.read_bytes()).hexdigest(), flush=True)
with tempfile.TemporaryDirectory(prefix="reserved-pads-") as directory:
    work = Path(directory)
    bench = work / "tb_reserved_pads.v"
    bench.write_text(BENCH)
    executable = work / "simulation"
    run(["iverilog", "-g2012", "-DNO_ICE40_DEFAULT_ASSIGNMENTS", "-s", "tb",
         "-I", str(ROOT / "firmware"), "-o", str(executable), str(bench),
         str(ROOT / "firmware/top.v"), str(cells)])
    run(["vvp", str(executable)])
