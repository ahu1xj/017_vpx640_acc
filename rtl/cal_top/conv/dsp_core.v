`include "all_defines.vh"

module dsp_core(
input                                   clk,
input                                   rst_n,
input                                   initial_valid,
input   [`DSP_CORE_OUTDATA_WIDTH-1:0]   initial_psum1,
input   [`DSP_CORE_OUTDATA_WIDTH-1:0]   initial_psum2,
input                                   data_valid,
input   signed [`DSP_CORE_INDATA_WIDTH-1:0]    fm_in1,
input   signed [`DSP_CORE_INDATA_WIDTH-1:0]    fm_in2,
input   signed [`DSP_CORE_INDATA_WIDTH-1:0]    weight_in,
output  reg signed [`DSP_CORE_OUTDATA_WIDTH-1:0]   fm_out1,
output  reg signed [`DSP_CORE_OUTDATA_WIDTH-1:0]   fm_out2
);
reg  signed [3*`DSP_CORE_INDATA_WIDTH:0] ad;
// ad = (fm_in1<<16)+fm_in2: preadd moved from DSP pre-adder to fabric (25b add in
// the nearly-empty operand cycle); multiply cycle is a preadder-free 25x8 DSP.
// Bit-identical to the old (d+a)*b; pipeline timing unchanged. (timing rework)
reg  signed [`DSP_CORE_INDATA_WIDTH-1:0] b;
reg         data_valid_r1;//first timecycle finish p value calculate
reg         p_valid;
reg         initial_valid_s1;
reg         initial_valid_s2;
reg  signed [`DSP_CORE_OUTDATA_WIDTH:0]   p; //p=ad*b
wire signed [`DSP_CORE_OUTDATA_WIDTH-1:0] psum1;
wire signed [`DSP_CORE_OUTDATA_WIDTH-1:0] psum2;

always @(posedge clk or negedge rst_n)
if(!rst_n)
	data_valid_r1<=1'b0;
else if(data_valid)
	data_valid_r1<=1'b1;
else
	data_valid_r1<=1'b0;

always @(posedge clk)
begin
    ad <= (fm_in1<<16) + fm_in2;
    b <= weight_in;
end

always @(posedge clk or negedge rst_n)
begin
    if(!rst_n)begin
        initial_valid_s1 <= 0;
        initial_valid_s2 <= 0;
    end
    else begin
        initial_valid_s1 <= initial_valid;
        initial_valid_s2 <= initial_valid_s1;
    end
end
always @(posedge clk)
begin
    p <= ad * b;
end

assign psum1=(initial_valid_s2)?initial_psum1:fm_out1;
assign psum2=(initial_valid_s2)?initial_psum2:fm_out2;

always @(posedge clk or negedge rst_n)
begin
    if(!rst_n)begin
        p_valid <= 0;
    end
    else begin
        p_valid <= data_valid_r1;
    end
end

always @(posedge clk)
begin
    if(p_valid) begin
        fm_out1 <= psum1 + $signed(p[31-:16]) + $signed({1'b0, p[15]});//if low data is negative high data should add 1
        fm_out2 <= psum2 + $signed(p[0+:16]);
    end
end

endmodule
