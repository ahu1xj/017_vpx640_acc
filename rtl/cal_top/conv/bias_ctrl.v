`include "all_defines.vh"

module bias_ctrl(
input                                           clk,
input                                           rst_n,
input                                           rdcmd_trans_valid,
input       [`BA_CTRL_INCMD-1:0]                rdcmd_trans,
input                                           s_axi_bias_tvalid,
input       [`BA_CTRL_BAINDATA_WIDTH-1:0]       s_axi_bias_tdata,
output                                          s_axi_bias_tready,
output                                          bias_valid,
output      [`BA_CTRL_BAOUTDATA_WIDTH-1:0]      bias_data
);
reg                                         tready;
reg     [`BA_CTRL_BAOUTDATA_WIDTH-1:0]      data;
reg     [1:0]                               count;

//Handshakecount
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin
        count <= 0;
    end
    else if(s_axi_bias_tvalid & s_axi_bias_tready) begin
        count <= count + 1; 
    end     
end
//s_axi_bias_tready 
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        tready <= 0;
    end
    else if(rdcmd_trans_valid & rdcmd_trans[`RD_MODEL_SEL_START+:`RD_MODEL_SEL_WIDTH] == `MODEL_SEL_CONV)begin
        tready <= 1;
    end
    else if(s_axi_bias_tvalid & s_axi_bias_tready & count ==2'b11)begin
        tready <= 0;
    end
end
assign s_axi_bias_tready = tready;
assign bias_valid        = ~tready;

//bias_data
always @(posedge clk) begin
    if(s_axi_bias_tvalid & s_axi_bias_tready) begin
        data <= {s_axi_bias_tdata , data[`BA_CTRL_BAOUTDATA_WIDTH-1:512]}; 
    end     
end
assign bias_data = data;
endmodule
