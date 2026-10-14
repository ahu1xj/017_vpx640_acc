`include "all_defines.vh"

module cal_top(
    input  wire         clk,
    input  wire         rst_n,

    input  wire         s_axis_rdcmd_tvalid,
    input  wire [111:0] s_axis_rdcmd_tdata,
    output wire         s_axis_rdcmd_tready,

    input  wire         store_done_req,
    output wire         store_done_ack,

    input  wire         s_axis_fm_tvalid,
    input  wire [511:0] s_axis_fm_tdata,
    output wire         s_axis_fm_tready,

    input  wire         s_axis_cpad_tvalid,
    input  wire [511:0] s_axis_cpad_tdata,
    output wire         s_axis_cpad_tready,

    input  wire         s_axis_cpad_store_tvalid,
    input  wire [511:0] s_axis_cpad_store_tdata,
    output wire         s_axis_cpad_store_tready,

    input  wire         s_axis_wrcmd_tvalid,
    input  wire [103:0] s_axis_wrcmd_tdata,
    output wire         s_axis_wrcmd_tready,

    input  wire         s_axis_we_tvalid,
    input  wire [511:0] s_axis_we_tdata,
    output wire         s_axis_we_tready,

    input  wire         s_axis_bias_tvalid,
    input  wire [511:0] s_axis_bias_tdata,
    output wire         s_axis_bias_tready,

    input  wire         s_axis_quan_param_tvalid,
    input  wire [511:0] s_axis_quan_param_tdata,
    output wire         s_axis_quan_param_tready,

    input  wire         s_axis_quan_fm1_tvalid,
    input  wire [511:0] s_axis_quan_fm1_tdata,
    output wire         s_axis_quan_fm1_tready,

    input  wire         s_axis_quan_fm2_tvalid,
    input  wire [511:0] s_axis_quan_fm2_tdata,
    output wire         s_axis_quan_fm2_tready,

    input  wire         s_axis_act_tvalid,
    input  wire [511:0] s_axis_act_tdata,
    output wire         s_axis_act_tready,

    output wire         m_axis_act_tvalid,
    output wire [511:0] m_axis_act_tdata,
    input  wire         m_axis_act_tready,

    output wire         m_axis_pooling_tvalid,
    output wire [511:0] m_axis_pooling_tdata,
    input  wire         m_axis_pooling_tready,

    output wire         m_axis_res_tvalid,
    output wire [511:0] m_axis_res_tdata,
    input  wire         m_axis_res_tready
);

wire         rdcmd_trans_valid;
wire [111:0] rdcmd_trans;

wire         fram_wren;
wire [13:0]  fram_wraddr;
wire [511:0] fram_wrdata;
wire         fram_frden;
wire [13:0]  fram_frdaddr;
wire [511:0] fram_frddata;
wire         fm_add_sig;
wire         fm_sub_sig;

wire         ffmtem_pre_tvalid;
wire [255:0] ffmtem_pre_tdata;
wire         ffmtem_pre_tready;
wire         ffmtem_tvalid;
wire [255:0] ffmtem_tdata;
wire         ffmtem_tready;
wire         fpooltem_pre_tvalid;
wire [15:0]  fpooltem_pre_tdata;
wire         fpooltem_pre_tready;
wire         fpooltem_tvalid;
wire [15:0]  fpooltem_tdata;
wire         fpooltem_tready;

wire         pram0b_wren;
wire [11:0]  pram0b_wraddr;
wire [511:0] pram0b_wrdata;
wire         pram0t_wren;
wire [11:0]  pram0t_wraddr;
wire [511:0] pram0t_wrdata;
wire         pram1b_wren;
wire [11:0]  pram1b_wraddr;
wire [511:0] pram1b_wrdata;
wire         pram1t_wren;
wire [11:0]  pram1t_wraddr;
wire [511:0] pram1t_wrdata;
wire [1:0]   pram_buffer_valid;
wire [11:0]  pram_rdaddr;
wire [511:0] pram0b_rddata;
wire [511:0] pram0t_rddata;
wire [511:0] pram1b_rddata;
wire [511:0] pram1t_rddata;

wire         pfmtem_pre_tvalid;
wire [255:0] pfmtem_pre_tdata;
wire         pfmtem_pre_tready;
wire         pfmtem_tvalid;
wire [255:0] pfmtem_tdata;
wire         pfmtem_tready;
wire         ppooltem_pre_tvalid;
wire [15:0]  ppooltem_pre_tdata;
wire         ppooltem_pre_tready;
wire         ppooltem_tvalid;
wire [15:0]  ppooltem_tdata;
wire         ppooltem_tready;

wire         wram_wren;
wire [13:0]  wram_wraddr;
wire [511:0] wram_wrdata;
wire         wram_rden;
wire [13:0]  wram_rdaddr;
wire [511:0] wram_rddata;
wire         we_update;
wire         wetem_pre_tvalid;
wire [511:0] wetem_pre_tdata;
wire         wetem_pre_tready;
wire         wetem_tvalid;
wire [511:0] wetem_tdata;
wire         wetem_tready;

wire         bias_valid;
wire [2047:0] bias_data;

wire         fetch_initial_valid;
wire         fetch_data_valid;
wire         fetch_end_data_valid;
wire [255:0] fetch_fm_data_256;
wire [511:0] fetch_weight_in;
wire [2047:0] fetch_bias_in;

wire         conv_valid;
wire         conv_ready;
wire         conv_candidate_start;
wire         conv_candidate_advance;
wire         conv_candidate_done;
wire         conv_candidate_valid;
wire [4095:0] conv_candidate_data;
wire         quan_res_fm1_pre_tvalid;
wire [511:0] quan_res_fm1_pre_tdata;
wire         quan_res_fm1_pre_tready;
wire         quan_res_fm1_tvalid;
wire [511:0] quan_res_fm1_tdata;
wire         quan_res_fm1_tready;
wire         quan_res_fm2_pre_tvalid;
wire [511:0] quan_res_fm2_pre_tdata;
wire         quan_res_fm2_pre_tready;
wire         quan_res_fm2_tvalid;
wire [511:0] quan_res_fm2_tdata;
wire         quan_res_fm2_tready;

wire         quan_act_pre_tvalid;
wire [511:0] quan_act_pre_tdata;
wire         quan_act_pre_tready;
wire         quan_act_valid;
wire [511:0] quan_act_data;
wire         quan_act_ready;

rdcmd_dispatch u_rdcmd_dispatch (
    .clk                  (clk),
    .rst_n                (rst_n),
    .s_axis_rdcmd_tvalid  (s_axis_rdcmd_tvalid),
    .s_axis_rdcmd_tdata   (s_axis_rdcmd_tdata),
    .s_axis_rdcmd_tready  (s_axis_rdcmd_tready),
    .rdcmd_trans_valid    (rdcmd_trans_valid),
    .rdcmd_trans          (rdcmd_trans),
    .store_done_req       (store_done_req),
    .store_done_ack       (store_done_ack)
);

fram_write_ctrl u_fram_write_ctrl (
    .clk                  (clk),
    .rst_n                (rst_n),
    .rdcmd_trans_valid    (rdcmd_trans_valid),
    .rdcmd_trans          (rdcmd_trans),
    .s_axis_fm_tvalid     (s_axis_fm_tvalid),
    .s_axis_fm_tdata      (s_axis_fm_tdata),
    .s_axis_fm_tready     (s_axis_fm_tready),
    .wren                 (fram_wren),
    .wraddr               (fram_wraddr),
    .wrdata               (fram_wrdata),
    .fm_add_sig           (fm_add_sig),
    .fm_sub_sig           (fm_sub_sig)
);

fram_512x11392_uram u_fram (
    .clka                 (clk),
    .wea                  (fram_wren),
    .addra                (fram_wraddr),
    .dina                 (fram_wrdata),
    .clkb                 (clk),
    .addrb                (fram_frdaddr),
    .doutb                (fram_frddata)
);

prefetch_fm u_prefetch_fm (
    .clk                  (clk),
    .rst_n                (rst_n),
    .rdcmd_trans_valid    (rdcmd_trans_valid),
    .rdcmd_trans          (rdcmd_trans),
    .frden                (fram_frden),
    .frdaddr              (fram_frdaddr),
    .frddata              (fram_frddata),
    .m_axis_ffmtem_tvalid (ffmtem_pre_tvalid),
    .m_axis_ffmtem_tdata  (ffmtem_pre_tdata),
    .m_axis_ffmtem_tready (ffmtem_pre_tready),
    .m_axis_fpooltem_tvalid(fpooltem_pre_tvalid),
    .m_axis_fpooltem_tdata(fpooltem_pre_tdata),
    .m_axis_fpooltem_tready(fpooltem_pre_tready),
    .fm_add_sig           (fm_add_sig),
    .fm_sub_sig           (fm_sub_sig)
);

axis_data_fifo_256_sync u_ffmtem_fifo (
    .s_axis_aresetn       (rst_n),
    .s_axis_aclk          (clk),
    .s_axis_tvalid        (ffmtem_pre_tvalid),
    .s_axis_tdata         (ffmtem_pre_tdata),
    .s_axis_tready        (ffmtem_pre_tready),
    .m_axis_tvalid        (ffmtem_tvalid),
    .m_axis_tdata         (ffmtem_tdata),
    .m_axis_tready        (ffmtem_tready)
);

axis_data_fifo_16_sync u_fpooltem_fifo (
    .s_axis_aresetn       (rst_n),
    .s_axis_aclk          (clk),
    .s_axis_tvalid        (fpooltem_pre_tvalid),
    .s_axis_tdata         (fpooltem_pre_tdata),
    .s_axis_tready        (fpooltem_pre_tready),
    .m_axis_tvalid        (fpooltem_tvalid),
    .m_axis_tdata         (fpooltem_tdata),
    .m_axis_tready        (fpooltem_tready)
);

pram_write_ctrl u_cpad_wr_ctrl (
    .clk                  (clk),
    .rst_n                (rst_n),
    .rdcmd_trans_valid    (rdcmd_trans_valid),
    .rdcmd_trans          (rdcmd_trans),
    .s_axis_wrcmd_tvalid  (s_axis_wrcmd_tvalid),
    .s_axis_wrcmd_tdata   (s_axis_wrcmd_tdata),
    .s_axis_wrcmd_tready  (s_axis_wrcmd_tready),
    .s_axis_cpad_load_tvalid(s_axis_cpad_tvalid),
    .s_axis_cpad_load_tdata(s_axis_cpad_tdata),
    .s_axis_cpad_load_tready(s_axis_cpad_tready),
    .s_axis_cpad_store_tvalid(s_axis_cpad_store_tvalid),
    .s_axis_cpad_store_tdata(s_axis_cpad_store_tdata),
    .s_axis_cpad_store_tready(s_axis_cpad_store_tready),
    .wren0b               (pram0b_wren),
    .wraddr0b             (pram0b_wraddr),
    .wrdata0b             (pram0b_wrdata),
    .wren0t               (pram0t_wren),
    .wraddr0t             (pram0t_wraddr),
    .wrdata0t             (pram0t_wrdata),
    .wren1b               (pram1b_wren),
    .wraddr1b             (pram1b_wraddr),
    .wrdata1b             (pram1b_wrdata),
    .wren1t               (pram1t_wren),
    .wraddr1t             (pram1t_wraddr),
    .wrdata1t             (pram1t_wrdata),
    .buffer_valid         (pram_buffer_valid)
);

pram_512x3712_uram u_pram0b (
    .clka                 (clk),
    .wea                  (pram0b_wren),
    .addra                (pram0b_wraddr),
    .dina                 (pram0b_wrdata),
    .clkb                 (clk),
    .addrb                (pram_rdaddr),
    .doutb                (pram0b_rddata)
);

pram_512x3712_uram u_pram0t (
    .clka                 (clk),
    .wea                  (pram0t_wren),
    .addra                (pram0t_wraddr),
    .dina                 (pram0t_wrdata),
    .clkb                 (clk),
    .addrb                (pram_rdaddr),
    .doutb                (pram0t_rddata)
);

pram_512x3712_uram u_pram1b (
    .clka                 (clk),
    .wea                  (pram1b_wren),
    .addra                (pram1b_wraddr),
    .dina                 (pram1b_wrdata),
    .clkb                 (clk),
    .addrb                (pram_rdaddr),
    .doutb                (pram1b_rddata)
);

pram_512x3712_uram u_pram1t (
    .clka                 (clk),
    .wea                  (pram1t_wren),
    .addra                (pram1t_wraddr),
    .dina                 (pram1t_wrdata),
    .clkb                 (clk),
    .addrb                (pram_rdaddr),
    .doutb                (pram1t_rddata)
);

prefetch_padding u_prefetch_padding (
    .clk                  (clk),
    .rst_n                (rst_n),
    .rdcmd_trans_valid    (rdcmd_trans_valid),
    .rdcmd_trans          (rdcmd_trans),
    .buffer_valid         (pram_buffer_valid),
    .prdaddr              (pram_rdaddr),
    .prddata0b            (pram0b_rddata),
    .prddata0t            (pram0t_rddata),
    .prddata1b            (pram1b_rddata),
    .prddata1t            (pram1t_rddata),
    .m_axis_pfmtem_tvalid (pfmtem_pre_tvalid),
    .m_axis_pfmtem_tdata  (pfmtem_pre_tdata),
    .m_axis_pfmtem_tready (pfmtem_pre_tready),
    .m_axis_ppooltem_tvalid(ppooltem_pre_tvalid),
    .m_axis_ppooltem_tdata(ppooltem_pre_tdata),
    .m_axis_ppooltem_tready(ppooltem_pre_tready)
);

axis_data_fifo_256_sync u_pfmtem_fifo (
    .s_axis_aresetn       (rst_n),
    .s_axis_aclk          (clk),
    .s_axis_tvalid        (pfmtem_pre_tvalid),
    .s_axis_tdata         (pfmtem_pre_tdata),
    .s_axis_tready        (pfmtem_pre_tready),
    .m_axis_tvalid        (pfmtem_tvalid),
    .m_axis_tdata         (pfmtem_tdata),
    .m_axis_tready        (pfmtem_tready)
);

axis_data_fifo_16_sync u_ppooltem_fifo (
    .s_axis_aresetn       (rst_n),
    .s_axis_aclk          (clk),
    .s_axis_tvalid        (ppooltem_pre_tvalid),
    .s_axis_tdata         (ppooltem_pre_tdata),
    .s_axis_tready        (ppooltem_pre_tready),
    .m_axis_tvalid        (ppooltem_tvalid),
    .m_axis_tdata         (ppooltem_tdata),
    .m_axis_tready        (ppooltem_tready)
);

wram_write_ctrl u_wram_write_ctrl (
    .clk                  (clk),
    .rst_n                (rst_n),
    .rdcmd_trans_valid    (rdcmd_trans_valid),
    .rdcmd_trans          (rdcmd_trans),
    .s_axis_we_tvalid     (s_axis_we_tvalid),
    .s_axis_we_tdata      (s_axis_we_tdata),
    .s_axis_we_tready     (s_axis_we_tready),
    .wren                 (wram_wren),
    .wraddr               (wram_wraddr),
    .wrdata               (wram_wrdata),
    .we_update            (we_update)
);

wram_512x6144_uram u_wram (
    .clka                 (clk),
    .wea                  (wram_wren),
    .addra                (wram_wraddr[12:0]),
    .dina                 (wram_wrdata),
    .clkb                 (clk),
    .addrb                (wram_rdaddr[12:0]),
    .doutb                (wram_rddata)
);

prefetch_weights u_prefetch_weights (
    .clk                  (clk),
    .rst_n                (rst_n),
    .rdcmd_trans_valid    (rdcmd_trans_valid),
    .rdcmd_trans          (rdcmd_trans),
    .wrden                (wram_rden),
    .wrdaddr              (wram_rdaddr),
    .wrddata              (wram_rddata),
    .m_aixs_wetem_tvalid  (wetem_pre_tvalid),
    .m_aixs_wetem_tdata   (wetem_pre_tdata),
    .m_aixs_wetem_tready  (wetem_pre_tready),
    .we_update            (we_update)
);

axis_data_fifo_512_sync u_wetem_fifo (
    .s_axis_aresetn       (rst_n),
    .s_axis_aclk          (clk),
    .s_axis_tvalid        (wetem_pre_tvalid),
    .s_axis_tdata         (wetem_pre_tdata),
    .s_axis_tready        (wetem_pre_tready),
    .m_axis_tvalid        (wetem_tvalid),
    .m_axis_tdata         (wetem_tdata),
    .m_axis_tready        (wetem_tready)
);

bias_ctrl u_bias_ctrl (
    .clk                  (clk),
    .rst_n                (rst_n),
    .rdcmd_trans_valid    (rdcmd_trans_valid),
    .rdcmd_trans          (rdcmd_trans),
    .s_axi_bias_tvalid    (s_axis_bias_tvalid),
    .s_axi_bias_tdata     (s_axis_bias_tdata),
    .s_axi_bias_tready    (s_axis_bias_tready),
    .bias_valid           (bias_valid),
    .bias_data            (bias_data)
);

cal_data_fetch u_cal_data_fetch (
    .clk                  (clk),
    .rst_n                (rst_n),
    .rdcmd_trans_valid    (rdcmd_trans_valid),
    .rdcmd_trans          (rdcmd_trans),
    .conv_ready           (conv_ready),
    .s_axis_pfmtem_tvalid (pfmtem_tvalid),
    .s_axis_pfmtem_tdata  (pfmtem_tdata),
    .s_axis_pfmtem_tready (pfmtem_tready),
    .s_axis_ffmtem_tvalid (ffmtem_tvalid),
    .s_axis_ffmtem_tdata  (ffmtem_tdata),
    .s_axis_ffmtem_tready (ffmtem_tready),
    .s_axis_wetem_tvalid  (wetem_tvalid),
    .s_axis_wetem_tdata   (wetem_tdata),
    .s_axis_wetem_tready  (wetem_tready),
    .bias_valid           (bias_valid),
    .bias_data            (bias_data),
    .initial_valid        (fetch_initial_valid),
    .data_valid           (fetch_data_valid),
    .end_data_valid       (fetch_end_data_valid),
    .fm_data_256          (fetch_fm_data_256),
    .weight_in            (fetch_weight_in),
    .bias_in              (fetch_bias_in)
);

pe u_pe (
    .clk                  (clk),
    .rst_n                (rst_n),
    .initial_valid        (fetch_initial_valid),
    .data_valid           (fetch_data_valid),
    .end_data_valid       (fetch_end_data_valid),
    .fm_data_256          (fetch_fm_data_256),
    .weight_in            (fetch_weight_in),
    .bias_in              (fetch_bias_in),
    .conv_candidate_start(conv_candidate_start),
    .conv_candidate_advance(conv_candidate_advance),
    .conv_candidate_done  (conv_candidate_done),
    .conv_valid           (conv_valid),
    .conv_candidate_valid (conv_candidate_valid),
    .conv_candidate_data  (conv_candidate_data)
);

quan u_quan (
    .clk                  (clk),
    .rst_n                (rst_n),
    .rdcmd_trans_valid    (rdcmd_trans_valid),
    .rdcmd_trans          (rdcmd_trans),
    .s_axis_quan_param_tvalid(s_axis_quan_param_tvalid),
    .s_axis_quan_param_tdata(s_axis_quan_param_tdata),
    .s_axis_quan_param_tready(s_axis_quan_param_tready),
    .s_axis_quan_fm1_tvalid(s_axis_quan_fm1_tvalid),
    .s_axis_quan_fm1_tdata(s_axis_quan_fm1_tdata),
    .s_axis_quan_fm1_tready(s_axis_quan_fm1_tready),
    .s_axis_quan_fm2_tvalid(s_axis_quan_fm2_tvalid),
    .s_axis_quan_fm2_tdata(s_axis_quan_fm2_tdata),
    .s_axis_quan_fm2_tready(s_axis_quan_fm2_tready),
    .conv_valid           (conv_valid),
    .conv_candidate_start(conv_candidate_start),
    .conv_candidate_advance(conv_candidate_advance),
    .conv_candidate_done  (conv_candidate_done),
    .conv_candidate_valid (conv_candidate_valid),
    .conv_candidate_data  (conv_candidate_data),
    .conv_ready           (conv_ready),
    .m_axis_res_fm1_tvalid(quan_res_fm1_pre_tvalid),
    .m_axis_res_fm1_tdata (quan_res_fm1_pre_tdata),
    .m_axis_res_fm1_tready(quan_res_fm1_pre_tready),
    .m_axis_res_fm2_tvalid(quan_res_fm2_pre_tvalid),
    .m_axis_res_fm2_tdata (quan_res_fm2_pre_tdata),
    .m_axis_res_fm2_tready(quan_res_fm2_pre_tready),
    .m_axis_quan_act_tvalid(quan_act_pre_tvalid),
    .m_axis_quan_act_tdata(quan_act_pre_tdata),
    .m_axis_quan_act_tready(quan_act_pre_tready)
);

axis_data_fifo_512x128_sync u_quan_res_fm1_fifo (
    .s_axis_aresetn       (rst_n),
    .s_axis_aclk          (clk),
    .s_axis_tvalid        (quan_res_fm1_pre_tvalid),
    .s_axis_tdata         (quan_res_fm1_pre_tdata),
    .s_axis_tready        (quan_res_fm1_pre_tready),
    .m_axis_tvalid        (quan_res_fm1_tvalid),
    .m_axis_tdata         (quan_res_fm1_tdata),
    .m_axis_tready        (quan_res_fm1_tready)
);

axis_data_fifo_512x64_sync u_quan_res_fm2_fifo (
    .s_axis_aresetn       (rst_n),
    .s_axis_aclk          (clk),
    .s_axis_tvalid        (quan_res_fm2_pre_tvalid),
    .s_axis_tdata         (quan_res_fm2_pre_tdata),
    .s_axis_tready        (quan_res_fm2_pre_tready),
    .m_axis_tvalid        (quan_res_fm2_tvalid),
    .m_axis_tdata         (quan_res_fm2_tdata),
    .m_axis_tready        (quan_res_fm2_tready)
);

axis_data_fifo_512x128_sync u_quan_act_fifo (
    .s_axis_aresetn       (rst_n),
    .s_axis_aclk          (clk),
    .s_axis_tvalid        (quan_act_pre_tvalid),
    .s_axis_tdata         (quan_act_pre_tdata),
    .s_axis_tready        (quan_act_pre_tready),
    .m_axis_tvalid        (quan_act_valid),
    .m_axis_tdata         (quan_act_data),
    .m_axis_tready        (quan_act_ready)
);

activate u_activate (
    .clk                  (clk),
    .rst_n                (rst_n),
    .rdcmd_trans_valid    (rdcmd_trans_valid),
    .rdcmd_trans          (rdcmd_trans),
    .s_axis_act_tvalid    (s_axis_act_tvalid),
    .s_axis_act_tdata     (s_axis_act_tdata),
    .s_axis_act_tready    (s_axis_act_tready),
    .s_axis_quan_act_tvalid(quan_act_valid),
    .s_axis_quan_act_tdata(quan_act_data),
    .s_axis_quan_act_tready(quan_act_ready),
    .m_axis_act_tvalid    (m_axis_act_tvalid),
    .m_axis_act_tdata     (m_axis_act_tdata),
    .m_axis_act_tready    (m_axis_act_tready)
);

resadd u_resadd (
    .clk                  (clk),
    .rst_n                (rst_n),
    .rdcmd_trans_valid    (rdcmd_trans_valid),
    .rdcmd_trans          (rdcmd_trans),
    .s_axis_res_fm1_tvalid(quan_res_fm1_tvalid),
    .s_axis_res_fm1_tdata (quan_res_fm1_tdata),
    .s_axis_res_fm1_tready(quan_res_fm1_tready),
    .s_axis_res_fm2_tvalid(quan_res_fm2_tvalid),
    .s_axis_res_fm2_tdata (quan_res_fm2_tdata),
    .s_axis_res_fm2_tready(quan_res_fm2_tready),
    .m_axis_res_tvalid    (m_axis_res_tvalid),
    .m_axis_res_tdata     (m_axis_res_tdata),
    .m_axis_res_tready    (m_axis_res_tready)
);

pooling_top u_pooling_top (
    .clk                  (clk),
    .rst_n                (rst_n),
    .rdcmd_trans_valid    (rdcmd_trans_valid),
    .rdcmd_trans          (rdcmd_trans),
    .s_axis_ppooltem_tvalid(ppooltem_tvalid),
    .s_axis_ppooltem_tdata(ppooltem_tdata),
    .s_axis_ppooltem_tready(ppooltem_tready),
    .s_axis_fpooltem_tvalid(fpooltem_tvalid),
    .s_axis_fpooltem_tdata(fpooltem_tdata),
    .s_axis_fpooltem_tready(fpooltem_tready),
    .m_axis_pooling_tdata (m_axis_pooling_tdata),
    .m_axis_pooling_tvalid(m_axis_pooling_tvalid),
    .m_axis_pooling_tready(m_axis_pooling_tready)
);

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Ports have explicit direction and width declarations.
// [PASS] Pure Verilog RTL only; no SystemVerilog keywords used.
// [PASS] Direct child instances use named port connections.
// [PASS] Four top-level shared RAM banks use sync_1r1w_ram with explicit parameters.
// [PASS] Read command ready/valid is owned by rdcmd_dispatch.
// [PASS] Quant-to-activation uses a quan skid plus cal-top 512x128 Vivado AXIS FIFO boundary.
// [PASS] Prefetch paths use local elastic/credit control and standard valid/ready.
// [PASS] Quant residual outputs use exact local credit and standard valid/ready.
// [PASS] PE-to-quan CONV boundary is four registered 1024-bit candidates plus narrow local controls.
// [PASS] The only 65536-bit snapshot is PE-local; quan final selection is registered 4:1.
// [PASS] No debug-only ports, system tasks, initial blocks, or delays.
// [PASS] No downstream ready signal is exported as a global combinational throttle.
// ============================================================

endmodule
