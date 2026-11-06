`timescale 1ns/1ps
module tb_clock_probe;
    reg clock = 0;
    reg reset_b = 0;
    real half_period = 4.0;
    integer input_edges = 0;
    wire [3:0] pmod;
    always #(half_period) clock = !clock;
    always @(posedge clock) input_edges = input_edges + 1;
    top dut (
        .i_glob_clock(clock), .i_rst_b(reset_b),
        .i_ss(1'b1), .i_sck(1'b0), .i_mosi(1'b0),
        .io_pmod_out(pmod), .io_pmod_in(4'b0000),
        .i_button(1'b0), .i_config(4'b0000),
        .i_iq_rx_09_p(1'b0), .i_iq_rx_24_n(1'b0),
        .i_iq_rx_clk_p(1'b0),
        .i_smi_a2(1'b0), .i_smi_a3(1'b0),
        .i_smi_soe_se(1'b1), .i_smi_swe_srw(1'b1)
    );

    task check_periods;
        input real expected_period;
        integer first_edge, cycle;
        realtime started, elapsed;
        begin
            @(posedge pmod[3]);
            first_edge = input_edges;
            started = $realtime;
            for (cycle = 0; cycle < 8; cycle = cycle + 1) begin
                @(posedge pmod[3]);
                elapsed = $realtime - started;
                if (input_edges - first_edge != 32)
                    $fatal(1, "Divider period was %0d input cycles", input_edges-first_edge);
                if (elapsed < expected_period - 0.05 || elapsed > expected_period + 0.05)
                    $fatal(1, "Period %0.3f ns, expected %0.3f ns", elapsed, expected_period);
                if (pmod[2:0] !== 3'bzzz)
                    $fatal(1, "Unselected PMOD pins are driven: %b", pmod[2:0]);
                first_edge = input_edges;
                started = $realtime;
                reset_b = !reset_b;
            end
            $display("PASS: /32 period %0.3f ns, other PMOD pads high impedance, resets independent", expected_period);
        end
    endtask

    initial begin
        check_periods(256.0);
        @(negedge clock);
        half_period = 13.468;
        check_periods(861.952);
        @(negedge clock);
        half_period = 13.477;
        check_periods(862.528);
        $finish;
    end
    initial begin
        #100000;
        $fatal(1, "Simulation timeout");
    end
endmodule
