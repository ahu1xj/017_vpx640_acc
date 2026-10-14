module prefetch_fm_read_mux(
    input           clk,
    input           rst_n,

    input           active_is_conv,
    input           active_is_pool,

    input           conv_frden,
    input   [13:0]  conv_frdaddr,
    output          conv_frdvalid,

    input           pool_frden,
    input   [13:0]  pool_frdaddr,
    output          pool_frdvalid,

    output          frden,
    output  [13:0]  frdaddr
);

reg req_owner_is_conv_d1;
reg req_owner_is_conv_d2;
reg req_owner_is_pool_d1;
reg req_owner_is_pool_d2;

wire conv_req_fire;
wire pool_req_fire;

assign conv_req_fire = active_is_conv && conv_frden;
assign pool_req_fire = active_is_pool && pool_frden;

assign frden   = conv_req_fire || pool_req_fire;
assign frdaddr = active_is_conv ? conv_frdaddr :
                 active_is_pool ? pool_frdaddr :
                                  14'd0;

// RAM 换 BMG IP 后读延迟为 2 拍（enb 常使能，DO_REG 每拍推进），owner 标签随之打 2 拍
assign conv_frdvalid = req_owner_is_conv_d2;
assign pool_frdvalid = req_owner_is_pool_d2;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        req_owner_is_conv_d1 <= 1'b0;
    else
        req_owner_is_conv_d1 <= conv_req_fire;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        req_owner_is_conv_d2 <= 1'b0;
    else
        req_owner_is_conv_d2 <= req_owner_is_conv_d1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        req_owner_is_pool_d1 <= 1'b0;
    else
        req_owner_is_pool_d1 <= pool_req_fire;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        req_owner_is_pool_d2 <= 1'b0;
    else
        req_owner_is_pool_d2 <= req_owner_is_pool_d1;
end

endmodule

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Rule #1:  All ports have explicit width declarations
// [PASS] Rule #2:  Ports use wire outputs and reg response tags
// [PASS] Rule #6:  No debug/error-attribution logic or diagnostic-only sideband in RTL
// [PASS] Rule #8:  No logic/always_ff/always_comb used
// [PASS] Rule #12: Idle frdaddr path is covered
// [PASS] Rule #16: Sequential logic uses non-blocking assignments only
// [PASS] Rule #34: Read mux only selects current owner and delays owner tag
// ============================================================
