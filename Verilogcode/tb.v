`timescale 1ns / 1ps

module top_tb;
    parameter N = 16;
    parameter Nnew = N + 6;

    reg clk;
    reg reset;
    wire [Nnew-1:0] X_out, Y_out;
    wire [N-1:0] Z_out,Z_temp;
    wire capture_div;
  //  wire [7:0] count_out;
    
    top #( .N(N)) uut (
        .clk(clk),
        .reset(reset),
        .X_out(X_out),
        .Y_out(Y_out),
        .Z_out(Z_out),
        .capture_div(capture_div)
    );
 
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ns clock period
    end 
    
    initial begin
        reset = 1'b1;
        #100;
        reset = 1'b0;
        #400; // Run the simulation for some time
        $finish;
    end
    
    initial begin
        $dumpfile("softmax_tb.vcd"); // Output VCD file
        $dumpvars(0, top_tb); // Dump all variables
    end
    
endmodule

