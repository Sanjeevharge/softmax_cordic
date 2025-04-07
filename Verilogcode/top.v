module top#(
    parameter N = 16,
    parameter Nnew = N+6
)(
    input clk,
    input reset,
    output [Nnew-1:0] X_out,
    output [Nnew-1:0] Y_out,
    output [N-1:0]  Z_out,
    output capture_div
);
    localparam STAGES_DIV = 8;
    localparam CLASSES = 6; 
    
    wire [Nnew-1:0] X_in, Y_in;
    wire [N-1:0] Z_in;
//    wire [Nnew-1:0] X_out, Y_out;
//    wire [N-1:0]  Z_out;
 //   wire capture_div;
    wire valid_div;
    wire temp;
      
    exp #(.N(N)) driver2 (
        .clk(clk),
        .reset(reset),
	    .r_out(temp),
        .X(X_in),
        .Y(Y_in),
	    .Z(Z_in),
        .valid_div(valid_div)
    );
    
    cordic_pipeline_div #(.N(N), .STAGES_DIV(STAGES_DIV)) pipeline (
        .clk(clk),
        .valid(valid_div),
        .X_in(X_in),
        .Y_in(Y_in),
        .Z_in(Z_in),
        .X_out(X_out),
        .Y_out(Y_out),
        .Z_out(Z_out),
        .capture(capture_div)
    );
    
endmodule

module exp#(
    parameter N = 16
)(
    input clk,
    input reset,
    output r_out,
    output reg [N+5:0] X,
    output reg [N+5:0] Y,
    output reg [N-1:0] Z,
    output reg valid_div
);
    localparam STAGES = 5;
    localparam CLASSES = 6; 
    
    wire [N-1:0] X_in, Y_in, Z_in;
    wire [N-1:0] X_out, Y_out, Z_out;
    wire capture;
    reg [N-1:0] captureReg;
    wire valid;
    reg [N-1:0] e_z;
    reg [N-1:0] e_z_values [0:CLASSES-1];
    reg [N+4:0] sum_all;
    reg [7:0] count;
    reg [7:0] count_out;

    input_driver #(.N(N)) driver (
        .clk(clk),
        .reset(reset),
        .X_out(X_in),
        .Y_out(Y_in),
        .Z_out(Z_in),
        .valid(valid)
    );
    
    cordic_pipeline #(.N(N), .STAGES(STAGES)) pipeline (
        .clk(clk),
        .valid(valid),
        .X_in(X_in),
        .Y_in(Y_in),
        .Z_in(Z_in),
        .X_out(X_out),
        .Y_out(Y_out),
        .Z_out(Z_out),
        .capture(capture)
    );
    
    always @(posedge clk or posedge reset) begin
        if(reset) begin
            count <= 0;
            count_out <= 0;
            sum_all <= 0;
    	end
        else begin
             if(r_out) begin    
                if (count_out < 6) begin
                    count_out <= count_out + 1;
                    valid_div <= 1'b1;  
                end else begin
                    valid_div <= 1'b0;  
                end
                    Y <= e_z_values[count_out];
                    X <= sum_all;
                    Z <= 16'b0;
              end	
            if (capture && (count < CLASSES)) begin
                e_z_values[count] <= e_z;
                sum_all <= sum_all + e_z;
                count <= count + 1;
            end
        end
    end 

    always @(X_out,Y_out,Z_out,(X_out+Y_out)) begin
    	e_z = X_out + Y_out;
    end

    assign r_out = (count == CLASSES) ? 1 : 0;
endmodule

module input_driver #(parameter N=16) (
    input  clk,
    input reset,
    output reg [N-1:0] X_out,
    output reg [N-1:0] Y_out,
    output reg [N-1:0] Z_out,
    output reg valid
);

reg [2:0] idx;
wire [N-1:0] X_mem [0:5];
wire [N-1:0] Y_mem [0:5];
wire [N-1:0] Z_mem [0:5];


always @(posedge clk or posedge reset) begin
        if(reset) begin
           idx <= 3'b000;
           valid <= 1'b0;
        end
        else if (idx < 6) begin
            idx   <= idx + 1;
	    valid <= 1'b1;  
        end else begin
            valid <= 1'b0;  
        end
end

always @(posedge clk or posedge reset) begin
    if (reset) begin
        X_out <= 0;
        Y_out <= 0;
        Z_out <= 0;
    end
    else begin
//  if(!reset) begin
        X_out <= X_mem[idx];
        Y_out <= Y_mem[idx];
        Z_out <= Z_mem[idx];
   end
end

assign X_mem[0] = 16'h26A3;
assign Y_mem[0] = 16'h0000;
assign Z_mem[0] = 16'h1000; 
assign X_mem[1] = 16'h26A3;
assign Y_mem[1] = 16'h0000;
assign Z_mem[1] = 16'h0800; 
assign X_mem[2] = 16'h26A3;
assign Y_mem[2] = 16'h0000;
assign Z_mem[2] = 16'h1800;
assign X_mem[3] = 16'h26A3;
assign Y_mem[3] = 16'h0000;
assign Z_mem[3] = 16'h1800; 
assign X_mem[4] = 16'h26A3;
assign Y_mem[4] = 16'h0000;
assign Z_mem[4] = 16'h1000; 
assign X_mem[5] = 16'h26A3;
assign Y_mem[5] = 16'h0000;
assign Z_mem[5] = 16'h1000;

endmodule

module cordic_pipeline #(
    parameter N = 16,
    parameter STAGES = 5
)(
    input              clk,
    input             valid,
    input      [N-1:0] X_in,
    input      [N-1:0] Y_in,
    input      [N-1:0] Z_in,
    output     [N-1:0] X_out,
    output     [N-1:0] Y_out,
    output     [N-1:0] Z_out,
    output     capture
);

wire [N-1:0] X_pipe [0:STAGES];
wire [N-1:0] Y_pipe [0:STAGES];
wire [N-1:0] Z_pipe [0:STAGES];

assign X_pipe[1] = X_in;
assign Y_pipe[1] = Y_in;
assign Z_pipe[1] = Z_in;


genvar i;
generate
  for (i = 1; i < STAGES; i = i + 1) begin : stages
    cordic_iteration #(.N(N)) stage_i (
      .clk(clk),
      .X(X_pipe[i]),
      .Y(Y_pipe[i]),
      .Z(Z_pipe[i]),
      .shift_val(i[4:0]),
      .Xi(X_pipe[i+1]),
      .Yi(Y_pipe[i+1]),
      .Zi(Z_pipe[i+1])
    );
  end
endgenerate

reg [2*(STAGES-1)-1:0] valid_pipeline;
always @(posedge clk) begin
    valid_pipeline <= { valid_pipeline[2*(STAGES-1)-2:0], valid};
end

assign capture = valid_pipeline[2*(STAGES-1)-1];

assign X_out = X_pipe[STAGES];
assign Y_out = Y_pipe[STAGES];
assign Z_out = Z_pipe[STAGES];

endmodule

module cordic_iteration#(
    parameter N = 16
)(
	input clk,
	input [N-1:0]X,
	input [N-1:0]Y,
	input [N-1:0]Z,
	input [4:0]shift_val,
	output [N-1:0]Xi,
	output [N-1:0]Yi,
	output [N-1:0]Zi
);



wire [N-1:0] X_shift;
wire dnew;
wire [N-1:0] Y_shift;
wire [N-1:0] angle_i;
assign dnew = ~Z[N-1];


shift #(.N(N)) shift_inst_X (
    .Xin(X),
    .shiftval(shift_val),
    .X(X_shift)
);


shift #(.N(N)) shift_inst_Y (
    .Xin(Y),
    .shiftval(shift_val),
    .X(Y_shift)
);


wire [N-1:0] X_temp;
wire [N-1:0] Y_temp;
wire [N-1:0] Z_temp;


tan #(.N(N)) tan_x (
    .i(shift_val),
    .angle_i(angle_i)
);

addorsub #(.N(N)) addsub_x (
    .clk(clk),
    .a(X),
    .b(Y_shift),
    .d(dnew),
    .c(Xi)
);


addorsub #(.N(N)) addsub_y (
    .clk(clk),
    .a(Y),
    .b(X_shift),
    .d(dnew),
    .c(Yi)
);



addorsub #(.N(N)) addsub_z (
    .clk(clk),
    .a(Z),
    .b(angle_i),
    .d(Z[N-1]),
    .c(Zi)
);


//always @(posedge clk) begin
 //   Xi <= X_temp;
//    Yi <= Y_temp;
//    Zi <= Z_temp;
//end

endmodule




`timescale 1ns/1ps
module shift #(
    parameter N = 16
)(
//    input clk,
    input [N-1:0] Xin,
    input [4:0] shiftval,
    output [N-1:0] X
);

//always @(posedge clk) begin
//    X <= {Xin[N-1], (Xin[N-2:0] >> shiftval)};
//end

assign X = {Xin[N-1], (Xin[N-2:0] >> shiftval)};

endmodule



module tan #(
    parameter N = 16
)(
//    input clk,
    input [4:0] i,
    output [N-1:0] angle_i
);

wire [N-1:0] LUT [0:31];



assign   LUT[0]  = 16'h0000;
assign    LUT[1]  = 16'h1193;
assign    LUT[2]  = 16'h082E;
assign    LUT[3]  = 16'h0405;
assign    LUT[4]  = 16'h0200;
assign    LUT[5]  = 16'h0100;
assign    LUT[6]  = 16'h0080;
assign    LUT[7]  = 16'h0040;
assign    LUT[8]  = 16'h0020;
assign    LUT[9]  = 16'h0010;
assign    LUT[10] = 16'h0008;
assign    LUT[11] = 16'h0004;
assign    LUT[12] = 16'h0002;
assign    LUT[13] = 16'h0001;
assign    LUT[14] = 16'h0001;
assign    LUT[15] = 16'h0000;
assign    LUT[16] = 16'h0000;
assign    LUT[17] = 16'h0000;
assign    LUT[18] = 16'h0000;
assign    LUT[19] = 16'h0000;
assign    LUT[20] = 16'h0000;
assign    LUT[21] = 16'h0000;
assign    LUT[22] = 16'h0000;
assign    LUT[23] = 16'h0000;
assign    LUT[24] = 16'h0000;
assign    LUT[25] = 16'h0000;
 assign   LUT[26] = 16'h0000;
 assign   LUT[27] = 16'h0000;
assign    LUT[28] = 16'h0000;
assign    LUT[29] = 16'h0000;
assign    LUT[30] = 16'h0000;
assign    LUT[31] = 16'h0000;


//always @(posedge clk) begin
//    angle_i <= LUT[i];
//end

assign angle_i = LUT[i];

endmodule






module addorsub #(
    parameter N = 16  
)(
	input clk,
	input [N-1:0]a,
	input [N-1:0]b,
	input d,
	output reg [N-1:0]c
);

wire a_sign;
wire b_sign;
wire [N-2:0] a_val = a[N-2:0];
wire [N-2:0] b_val = b[N-2:0];
assign a_sign = a[N-1];
assign b_sign = b[N-1];

reg c_sign;
reg [N-2:0] c_val;


always @(posedge clk)
begin
	if(d) begin
		if (a_sign == b_sign) begin
           		c_val  <= a_val + b_val;
            		c_sign <= a_sign;
        	end

		else begin
           	   if (a_val >= b_val) begin
                	c_val  <= a_val - b_val;
                	c_sign <= a_sign;
                   end
                   else begin
                	c_val  <= b_val - a_val;
               		c_sign <= b_sign;
                   end
                end
	end


	else begin 
		if (a_sign != b_sign) begin
            		c_val  <= a_val + b_val;
           		c_sign <= a_sign;
        	end

		else begin
            		if (a_val >= b_val) begin
                		c_val  <= a_val - b_val;
                		c_sign <= a_sign;
            		end
            		else begin
                		c_val  <= b_val - a_val;
                		c_sign <= ~a_sign;
            		end
    		end
	end
	c = {c_sign, c_val};
end


endmodule




module cordic_pipeline_div #(
    parameter N = 16,
    parameter Nnew = N+6,
    parameter STAGES_DIV = 8
)(
    input              clk,
    input             valid,
    input      [Nnew-1:0] X_in,
    input      [Nnew-1:0] Y_in,
    input      [N-1:0] Z_in,
    output     [Nnew-1:0] X_out,
    output     [Nnew-1:0] Y_out,
    output     [N-1:0] Z_out,
    output     capture
);

wire [Nnew-1:0] X_pipe [0:STAGES_DIV];
wire [Nnew-1:0] Y_pipe [0:STAGES_DIV];
wire [N-1:0] Z_pipe [0:STAGES_DIV];

assign X_pipe[1] = X_in;
assign Y_pipe[1] = Y_in;
assign Z_pipe[1] = Z_in;


genvar i;
generate
  for (i = 1; i < STAGES_DIV; i = i + 1) begin : stages
    cordic_iteration_div #(.N(N)) stage_i (
      .clk(clk),
      .X(X_pipe[i]),
      .Y(Y_pipe[i]),
      .Z(Z_pipe[i]),
      .shift_val(i[4:0]),
      .Xi(X_pipe[i+1]),
      .Yi(Y_pipe[i+1]),
      .Zi(Z_pipe[i+1])
    );
  end
endgenerate

reg [2*(STAGES_DIV-1)-1:0] valid_pipeline;
always @(posedge clk) begin
    valid_pipeline <= { valid_pipeline[2*(STAGES_DIV-1)-2:0], valid};
end

assign capture = valid_pipeline[2*(STAGES_DIV-1)-1];

assign X_out = X_pipe[STAGES_DIV];
assign Y_out = Y_pipe[STAGES_DIV];
assign Z_out = Z_pipe[STAGES_DIV];

endmodule

module cordic_iteration_div#(
    parameter N = 16,
    parameter Nnew = N + 6
)(
	input clk,
	input [Nnew-1:0]X,
	input [Nnew-1:0]Y,
	input [N-1:0]Z,
	input [4:0]shift_val,
	output [Nnew-1:0]Xi,
	output [Nnew-1:0]Yi,
	output [N-1:0]Zi
);


wire [Nnew-1:0] X_shift;
wire d;
wire [N-1:0] angle_i;
wire dnew;


shift #(.N(Nnew)) shift_inst_X (
    .Xin(X),
    .shiftval(shift_val),
    .X(X_shift)
);

power #(.N(N)) power_x (
    .i(shift_val),
    .angle_i(angle_i)
);

addorsub #(.N(Nnew)) addsub_x (
    .clk(clk),
    .a(X),
    .b(22'd0),
    .d(d),
    .c(Xi)
);


addorsub #(.N(Nnew)) addsub_y (
    .clk(clk),
    .a(Y),
    .b(X_shift),
    .d(d),
    .c(Yi)
);



addorsub #(.N(N)) addsub_z (
    .clk(clk),
    .a(Z),
    .b(angle_i),
    .d(dnew),
    .c(Zi)
);

assign d = X[Nnew-1]^Y[Nnew-1];
assign dnew = ~d;

//always @(posedge clk) begin
 //   Xi <= X_temp;
//    Yi <= Y_temp;
//    Zi <= Z_temp;
//end

endmodule




module power #(
    parameter N = 16
)(
//    input clk,
    input [4:0] i,
    output [N-1:0] angle_i
);

wire [N-1:0] LUT [0:31];


assign    LUT[0]  = 16'h0000;
assign     LUT[1]  = 16'h1000;
assign     LUT[2]  = 16'h0800;
assign     LUT[3]  = 16'h0400;
assign     LUT[4]  = 16'h0200;
 assign    LUT[5]  = 16'h0100;
assign    LUT[6]  = 16'h0080;
assign    LUT[7]  = 16'h0040;
assign     LUT[8]  = 16'h0020;
assign     LUT[9]  = 16'h0010;
assign     LUT[10] = 16'h0008;
assign     LUT[11] = 16'h0004;
assign     LUT[12] = 16'h0002;
assign     LUT[13] = 16'h0001;
assign     LUT[14] = 16'h0001;
assign     LUT[15] = 16'h0000;
assign     LUT[16] = 16'h0000;
assign     LUT[17] = 16'h0000;
assign     LUT[18] = 16'h0000;
assign     LUT[19] = 16'h0000;
assign     LUT[20] = 16'h0000;
assign     LUT[21] = 16'h0000;
assign     LUT[22] = 16'h0000;
assign     LUT[23] = 16'h0000;
assign     LUT[24] = 16'h0000;
assign     LUT[25] = 16'h0000;
assign     LUT[26] = 16'h0000;
assign     LUT[27] = 16'h0000;
assign     LUT[28] = 16'h0000;
 assign    LUT[29] = 16'h0000;
 assign    LUT[30] = 16'h0000;
 assign    LUT[31] = 16'h0000;


assign angle_i = LUT[i];

endmodule






