module tensor_processing_unit #(
    parameter integer AXI_DATA_WIDTH    = 64,       // AXI 数据总线宽度
    parameter integer AXI_ADDR_WIDTH    = 32,       // AXI 地址总线宽度
    parameter integer AXI_ID_WIDTH      = 1,        // AXI ID 宽度
    parameter integer AXI_USER_WIDTH    = 1,        // AXI 用户信号宽度
    parameter integer PE_DATA_WIDTH     = 32,       // PE/Core 数据元素宽度
    parameter integer ACCUM_WIDTH       = 32,       // 累加器/输出数据 D 宽度
    parameter integer INPUT_BUF_DEPTH   = 1024,     // 每个输入缓冲区 (A, B, C) 的最大元素数
    parameter integer OUTPUT_BUF_DEPTH  = 1024,     // 输出缓冲区 (D) 的最大元素数
    parameter integer CORE_ADDR_WIDTH   = $clog2(INPUT_BUF_DEPTH), // 内部缓冲区地址宽度，可能需要重新评估
    parameter integer PE_ARRAY_HEIGHT   = 16,       // PE 阵列高度 (M 维度相关)
    parameter integer PE_ARRAY_WIDTH    = 16,       // PE 阵列宽度 (N 维度相关)
    parameter integer MATRIX_SIZE_WIDTH = 32        // 矩阵尺寸编码宽度
) (
    // 时钟和复位
    input                                     aclk,               // AXI 时钟
    input                                     aresetn,            // AXI 复位 (低有效)

    // AXI4-Lite 从接口 (控制/状态寄存器)
    input                                     s_axil_awvalid,     // 写地址有效
    output logic                              s_axil_awready,     // 写地址准备好
    input      [AXI_ADDR_WIDTH-1:0]           s_axil_awaddr,      // 写地址
    input                                     s_axil_wvalid,      // 写数据有效
    output logic                              s_axil_wready,      // 写数据准备好
    input      [31:0]                         s_axil_wdata,       // 写数据
    input      [3:0]                          s_axil_wstrb,       // 写选通
    output logic                              s_axil_bvalid,      // 写响应有效
    input                                     s_axil_bready,      // 写响应准备好
    output logic [1:0]                        s_axil_bresp,       // 写响应代码
    input                                     s_axil_arvalid,     // 读地址有效
    output logic                              s_axil_arready,     // 读地址准备好
    input      [AXI_ADDR_WIDTH-1:0]           s_axil_araddr,      // 读地址
    output logic                              s_axil_rvalid,      // 读数据有效
    input                                     s_axil_rready,      // 读数据准备好
    output logic [31:0]                       s_axil_rdata,       // 读数据
    output logic [1:0]                        s_axil_rresp,       // 读响应代码

    // AXI4-Full 主接口 (从内存读取输入数据 A, B, C)
    output logic [AXI_ID_WIDTH-1:0]           m_axi_read_arid,     // 读 ID
    output logic [AXI_ADDR_WIDTH-1:0]         m_axi_read_araddr,   // 读地址
    output logic [7:0]                        m_axi_read_arlen,    // 读突发长度
    output logic [2:0]                        m_axi_read_arsize,   // 读突发大小
    output logic [1:0]                        m_axi_read_arburst,  // 读突发类型
    output logic                              m_axi_read_arlock,   // 读锁定类型
    output logic [3:0]                        m_axi_read_arcache,  // 读缓存类型
    output logic [2:0]                        m_axi_read_arprot,   // 读保护类型
    output logic [3:0]                        m_axi_read_arqos,    // 读 QoS
    output logic                              m_axi_read_arvalid,  // 读地址有效
    input                                     m_axi_read_arready,  // 读地址准备好
    input      [AXI_ID_WIDTH-1:0]             m_axi_read_rid,      // 读数据 ID
    input      [AXI_DATA_WIDTH-1:0]           m_axi_read_rdata,    // 读数据
    input      [1:0]                          m_axi_read_rresp,    // 读响应代码
    input                                     m_axi_read_rlast,    // 读最后
    input                                     m_axi_read_rvalid,   // 读数据有效
    output logic                              m_axi_read_rready,   // 读数据准备好

    // AXI4-Full 主接口 (将输出张量 D 写回内存)
    output logic [AXI_ID_WIDTH-1:0]           m_axi_awid,         // 写 ID
    output logic [AXI_ADDR_WIDTH-1:0]         m_axi_awaddr,       // 写地址
    output logic [7:0]                        m_axi_awlen,        // 写突发长度
    output logic [2:0]                        m_axi_awsize,       // 写突发大小
    output logic [1:0]                        m_axi_awburst,      // 写突发类型
    output logic                              m_axi_awlock,       // 写锁定类型
    output logic [3:0]                        m_axi_awcache,      // 写缓存类型
    output logic [2:0]                        m_axi_awprot,       // 写保护类型
    output logic [3:0]                        m_axi_awqos,        // 写 QoS
    output logic                              m_axi_awvalid,      // 写地址有效
    input                                     m_axi_awready,      // 写地址准备好
    output logic [AXI_DATA_WIDTH-1:0]         m_axi_wdata,        // 写数据
    output logic [AXI_DATA_WIDTH/8-1:0]       m_axi_wstrb,        // 写选通
    output logic                              m_axi_wlast,        // 写最后
    output logic                              m_axi_wvalid,       // 写数据有效
    input                                     m_axi_wready,       // 写数据准备好
    input      [AXI_ID_WIDTH-1:0]             m_axi_bid,          // 写响应 ID
    input      [1:0]                          m_axi_bresp,        // 写响应代码
    input                                     m_axi_bvalid,       // 写响应有效
    output logic                              m_axi_bready        // 写响应准备好
);


    // --- 参数和本地参数 ---
	localparam integer TOTAL_PES = PE_ARRAY_HEIGHT * PE_ARRAY_WIDTH;
  
    localparam PE_ELEMENTS_PER_AXI = AXI_DATA_WIDTH / PE_DATA_WIDTH; // 每个 AXI 数据包含的 PE 元素数量
    localparam ACCUM_ELEMENTS_PER_AXI = AXI_DATA_WIDTH / ACCUM_WIDTH; // 每个 AXI 数据包含的累加器元素数量
    localparam MAX_BURST_LENGTH = 8'd255;     // AXI4规范支持的最大突发长度

    // 矩阵维度参数
    localparam M_BITS = 11;
    localparam N_BITS = 11;
    localparam K_BITS = 10;
    
    // AXI-Lite 寄存器地址映射
    localparam ADDR_CONTROL      = 32'h00; // 写: START(位0), RESET(位1) | 读: STATUS(位0=DONE)
    localparam ADDR_MATRIX_SIZE  = 32'h04; // 写/读: {M, N, K}
    localparam ADDR_OP_MODE      = 32'h08; // 写/读: [2:0]精度, [3]操作模式
    localparam ADDR_INPUT_A_ADDR = 32'h10; // 写/读: A 在外部存储器中的基地址
    localparam ADDR_INPUT_B_ADDR = 32'h14; // 写/读: B 的基地址
    localparam ADDR_INPUT_C_ADDR = 32'h18; // 写/读: C 的基地址
    localparam ADDR_OUTPUT_D_ADDR= 32'h1C; // 写/读: D 的基地址

    // --- 内部寄存器 (AXI-Lite 控制) ---
    logic                             control_reg_start;     // 启动计算触发器
    logic                             control_reg_reset;     // 软件复位触发器
    logic [MATRIX_SIZE_WIDTH-1:0]     matrix_size_reg;       // 矩阵尺寸寄存器
    logic [3:0]                       op_mode_reg;           // [2:0]精度模式, [3]是否加C
    logic [AXI_ADDR_WIDTH-1:0]        input_a_addr_reg;      // A 矩阵内存地址
    logic [AXI_ADDR_WIDTH-1:0]        input_b_addr_reg;      // B 矩阵内存地址
    logic [AXI_ADDR_WIDTH-1:0]        input_c_addr_reg;      // C 矩阵内存地址
    logic [AXI_ADDR_WIDTH-1:0]        output_d_addr_reg;     // D 矩阵内存地址
    logic                             status_reg_done;       // 计算完成标志

    // --- 内部缓冲区 ---
    logic [PE_DATA_WIDTH-1:0]  input_buffer_A [INPUT_BUF_DEPTH-1:0];
    logic [PE_DATA_WIDTH-1:0]  input_buffer_B [INPUT_BUF_DEPTH-1:0];
    logic [ACCUM_WIDTH-1:0]    input_buffer_C [INPUT_BUF_DEPTH-1:0];
    logic [ACCUM_WIDTH-1:0]    output_buffer_D [OUTPUT_BUF_DEPTH-1:0];

    // --- 矩阵维度 ---
    logic [M_BITS-1:0] m_dim;
    logic [N_BITS-1:0] n_dim;
    logic [K_BITS-1:0] k_dim;
    logic [$clog2(INPUT_BUF_DEPTH):0] num_elements_A;
    logic [$clog2(INPUT_BUF_DEPTH):0] num_elements_B;
    logic [$clog2(INPUT_BUF_DEPTH):0] num_elements_C;
    logic [$clog2(OUTPUT_BUF_DEPTH):0] num_elements_D;
    logic [7:0] num_beats_A, num_beats_B, num_beats_C, num_beats_D_write;
    
    // --- AXI读取相关状态 ---
    logic axi_read_active;              // 读取事务活跃标志
    logic [7:0] axi_read_beat_count;    // 当前突发传输内的拍数计数
    logic [7:0] axi_read_expected_beats; // 预期的总拍数
    logic [CORE_ADDR_WIDTH:0] fetch_index; // 当前写入缓冲区的索引
    logic [AXI_ADDR_WIDTH-1:0] current_read_addr; // 当前读取地址
    
    // --- 主状态机相关 ---
    typedef enum logic [2:0] {
        IDLE,
        FETCH_A,
        FETCH_B,
        FETCH_C,
        COMPUTE,
        WRITE_BACK_D,
        WAIT_BVALID,
        DONE
    } main_state_t;
    main_state_t current_state, next_state;
    logic reset_l; // 内部高电平有效复位

    // --- 标志和计数器 ---
    logic fetch_a_done, fetch_b_done, fetch_c_done;
    logic compute_core_done;
    logic write_d_done;
    logic m_axi_write_active;
    logic [7:0] m_axi_write_beat_count;
    logic [CORE_ADDR_WIDTH:0] write_index;

    // --- 计算核心接口信号 ---
    logic compute_start_i;
    logic [CORE_ADDR_WIDTH-1:0] core_a_addr, core_b_addr, core_c_addr, core_d_addr;
    logic [ACCUM_WIDTH-1:0] core_d_data_out;
    logic core_d_write_enable;
	

    assign m_dim = matrix_size_reg[MATRIX_SIZE_WIDTH-1 : MATRIX_SIZE_WIDTH-M_BITS];
    assign n_dim = matrix_size_reg[MATRIX_SIZE_WIDTH-M_BITS-1 : MATRIX_SIZE_WIDTH-M_BITS-N_BITS];
    assign k_dim = matrix_size_reg[K_BITS-1 : 0];

    // 计算所需元素和 AXI 传输数量
    always_comb begin
        num_elements_A = m_dim * k_dim;
        num_elements_B = k_dim * n_dim;
        num_elements_C = op_mode_reg[3] ? (m_dim * n_dim) : 0; // 只有在加C模式下才需要C
        num_elements_D = m_dim * n_dim;
        num_beats_A = (num_elements_A + PE_ELEMENTS_PER_AXI - 1) / PE_ELEMENTS_PER_AXI;
        num_beats_B = (num_elements_B + PE_ELEMENTS_PER_AXI - 1) / PE_ELEMENTS_PER_AXI;
        num_beats_C = (num_elements_C > 0) ? ((num_elements_C + ACCUM_ELEMENTS_PER_AXI - 1) / ACCUM_ELEMENTS_PER_AXI) : 0;
        num_beats_D_write = (num_elements_D + ACCUM_ELEMENTS_PER_AXI - 1) / ACCUM_ELEMENTS_PER_AXI;
    end

    // --- 实例化计算核心 ---
    tensor_compute_core #(
        .NUM_PE(PE_ARRAY_HEIGHT * PE_ARRAY_WIDTH),
        .PE_DATA_WIDTH(PE_DATA_WIDTH),
        .ACCUM_WIDTH(ACCUM_WIDTH),
        .BUF_ADDR_WIDTH(CORE_ADDR_WIDTH),
        .MATRIX_SIZE_WIDTH(MATRIX_SIZE_WIDTH),
        .K_BITS(K_BITS)
    ) compute_core_inst (
        .clk(aclk),
        .reset(reset_l),
        .start_computation_pulse(compute_start_i),
        .matrix_size_i(matrix_size_reg),
        .operation_mode_i({1'b0, op_mode_reg[3]}), // [1:0] = [0, add_c_flag]
        .precision_mode_i(op_mode_reg[2:0]),       // [2:0] = 精度模式
        
        .a_addr_o(core_a_addr),
        .b_addr_o(core_b_addr),
        .c_addr_o(core_c_addr),
        .a_data_i(input_buffer_A[core_a_addr]),
        .b_data_i(input_buffer_B[core_b_addr]),
        .c_data_i(input_buffer_C[core_c_addr]),
        
        .d_addr_o(core_d_addr),
        .d_data_o(core_d_data_out),
        .d_write_enable_o(core_d_write_enable),
        
        .computation_done(compute_core_done)
    );

    // --- 复位逻辑 ---
    assign reset_l = ~aresetn; // 转换复位极性

    // --- AXI4-Lite 从接口实现 ---
    logic [AXI_ADDR_WIDTH-1:0] axil_read_addr_reg, axil_write_addr_reg;
    logic axil_awready_logic, axil_wready_logic, axil_arready_logic;
    logic axil_bvalid_logic, axil_rvalid_logic;

    // AW 通道
    assign s_axil_awready = axil_awready_logic;
    always_ff @(posedge aclk) begin
        if (reset_l) begin
            axil_awready_logic <= 1'b0;
            axil_write_addr_reg <= '0;
        end else begin
            if (!axil_awready_logic && s_axil_awvalid) begin
                axil_awready_logic <= 1'b1;
                axil_write_addr_reg <= s_axil_awaddr;
            end else if (s_axil_wvalid && axil_wready_logic) begin
                axil_awready_logic <= 1'b0;
            end
        end
    end

    // W 通道
    assign s_axil_wready = axil_wready_logic;
    always_ff @(posedge aclk) begin
        if (reset_l) begin
            axil_wready_logic <= 1'b0;
        end else begin
            if (axil_awready_logic && s_axil_awvalid && !axil_wready_logic && s_axil_wvalid) begin
                axil_wready_logic <= 1'b1;
            end else begin
                axil_wready_logic <= 1'b0;
            end
        end
    end

    // 写寄存器逻辑
    always_ff @(posedge aclk) begin
        if (reset_l) begin
            control_reg_start <= 1'b0;
            control_reg_reset <= 1'b0;
            matrix_size_reg <= '0;
            op_mode_reg <= '0;
            input_a_addr_reg <= '0;
            input_b_addr_reg <= '0;
            input_c_addr_reg <= '0;
            output_d_addr_reg <= '0;
        end else begin
            if (axil_awready_logic && s_axil_awvalid && axil_wready_logic && s_axil_wvalid) begin
                case (axil_write_addr_reg)
                    ADDR_CONTROL: begin
                        if (s_axil_wstrb[0]) control_reg_start <= s_axil_wdata[0];
                        if (s_axil_wstrb[0]) control_reg_reset <= s_axil_wdata[1];
                    end
                    ADDR_MATRIX_SIZE: if (s_axil_wstrb[3:0] == 4'b1111) matrix_size_reg <= s_axil_wdata;
                    ADDR_OP_MODE: if (s_axil_wstrb[0]) op_mode_reg <= s_axil_wdata[3:0]; // [2:0]=精度, [3]=add_c
                    ADDR_INPUT_A_ADDR: if (s_axil_wstrb[3:0] == 4'b1111) input_a_addr_reg <= s_axil_wdata;
                    ADDR_INPUT_B_ADDR: if (s_axil_wstrb[3:0] == 4'b1111) input_b_addr_reg <= s_axil_wdata;
                    ADDR_INPUT_C_ADDR: if (s_axil_wstrb[3:0] == 4'b1111) input_c_addr_reg <= s_axil_wdata;
                    ADDR_OUTPUT_D_ADDR: if (s_axil_wstrb[3:0] == 4'b1111) output_d_addr_reg <= s_axil_wdata;
                    default: ;
                endcase
            end
            
            // 清除标志
            if (status_reg_done || control_reg_reset) begin
                control_reg_start <= 1'b0;
            end
            if (control_reg_reset) begin
                control_reg_reset <= 1'b0; // 自清除
            end
        end
    end

    // B 通道
    assign s_axil_bvalid = axil_bvalid_logic;
    assign s_axil_bresp = 2'b00; // OKAY
    always_ff @(posedge aclk) begin
        if (reset_l) begin
            axil_bvalid_logic <= 1'b0;
        end else begin
            if (axil_wready_logic && s_axil_wvalid && !axil_bvalid_logic) begin
                axil_bvalid_logic <= 1'b1;
            end else if (s_axil_bready && axil_bvalid_logic) begin
                axil_bvalid_logic <= 1'b0;
            end
        end
    end

    // AR 通道
    assign s_axil_arready = axil_arready_logic;
    always_ff @(posedge aclk) begin
        if (reset_l) begin
            axil_arready_logic <= 1'b0;
            axil_read_addr_reg <= '0;
        end else begin
            if (!axil_arready_logic && s_axil_arvalid) begin
                axil_arready_logic <= 1'b1;
                axil_read_addr_reg <= s_axil_araddr;
            end else if (axil_rvalid_logic && s_axil_rready) begin
                axil_arready_logic <= 1'b0;
            end
        end
    end

    // R 通道
    assign s_axil_rvalid = axil_rvalid_logic;
    assign s_axil_rresp = 2'b00; // OKAY
    logic [31:0] axil_rdata_logic;

    assign s_axil_rdata = axil_rdata_logic;
    always_ff @(posedge aclk) begin
        if (reset_l) begin
            axil_rvalid_logic <= 1'b0;
            axil_rdata_logic <= '0;
        end else begin
            if (axil_arready_logic && s_axil_arvalid && !axil_rvalid_logic) begin
                axil_rvalid_logic <= 1'b1;
                case (axil_read_addr_reg)
                    ADDR_CONTROL:      axil_rdata_logic <= {31'b0, status_reg_done};
                    ADDR_MATRIX_SIZE:  axil_rdata_logic <= matrix_size_reg;
                    ADDR_OP_MODE:      axil_rdata_logic <= {28'b0, op_mode_reg};
                    ADDR_INPUT_A_ADDR: axil_rdata_logic <= input_a_addr_reg;
                    ADDR_INPUT_B_ADDR: axil_rdata_logic <= input_b_addr_reg;
                    ADDR_INPUT_C_ADDR: axil_rdata_logic <= input_c_addr_reg;
                    ADDR_OUTPUT_D_ADDR:axil_rdata_logic <= output_d_addr_reg;
                    default:           axil_rdata_logic <= 32'hDEADBEEF;
                endcase
            end else if (s_axil_rready && axil_rvalid_logic) begin
                axil_rvalid_logic <= 1'b0;
            end
        end
    end

    // --- AXI4-Full 主接口 (读 A, B, C) ---
    // 固定 AXI 读取配置
    assign m_axi_read_arburst = 2'b01; // INCR
    assign m_axi_read_arsize = $clog2(AXI_DATA_WIDTH/8); // 数据宽度对应的字节数
    assign m_axi_read_arlock = 1'b0;   // 普通访问
    assign m_axi_read_arcache = 4'b0011; // 普通缓冲、缓存访问
    assign m_axi_read_arprot = 3'b000; // 普通安全非特权访问
    assign m_axi_read_arqos = 4'b0000; // 默认QoS级别
    assign m_axi_read_arid = '0;       // 默认ID为0
	logic [7:0] remaining_beats;

    // AXI 读取地址通道控制逻辑
    always_ff @(posedge aclk) begin
        if (reset_l) begin
            m_axi_read_arvalid <= 1'b0;
            m_axi_read_araddr <= '0;
            m_axi_read_arlen <= '0;
            axi_read_active <= 1'b0;
            axi_read_expected_beats <= '0;
            current_read_addr <= '0;
        end else begin
            // 处理新的读取请求
            if (!axi_read_active && !m_axi_read_arvalid) begin
                if (current_state == FETCH_A && !fetch_a_done) begin
                    // 读取A矩阵
                    m_axi_read_arvalid <= 1'b1;
                    m_axi_read_araddr <= input_a_addr_reg + (fetch_index * PE_DATA_WIDTH / 8);//这里取的字节，fetch_index每次加PE_ELEMENTS_PER_AXI，所以刚好地址间隔一个beat
                    
                    // 计算剩余读取量，并确保不超过最大突发长度
                   
                    remaining_beats = num_beats_A - fetch_index / PE_ELEMENTS_PER_AXI;
                    
                    if (remaining_beats > MAX_BURST_LENGTH)
                        m_axi_read_arlen <= MAX_BURST_LENGTH; // 最大突发传输
                    else
                        m_axi_read_arlen <= remaining_beats - 1; // AXI协议：len = 传输次数 - 1
                    
                    current_read_addr <= input_a_addr_reg;
                end else if (current_state == FETCH_B && !fetch_b_done) begin
                    // 读取B矩阵
                    m_axi_read_arvalid <= 1'b1;
                    m_axi_read_araddr <= input_b_addr_reg + (fetch_index * PE_DATA_WIDTH / 8);
                    
                    // 计算剩余读取量，并确保不超过最大突发长度
                   
                    remaining_beats = num_beats_B - fetch_index / PE_ELEMENTS_PER_AXI;
                    
                    if (remaining_beats > MAX_BURST_LENGTH)
                        m_axi_read_arlen <= MAX_BURST_LENGTH;
                    else
                        m_axi_read_arlen <= remaining_beats - 1;
                    
                    current_read_addr <= input_b_addr_reg;
                end else if (current_state == FETCH_C && !fetch_c_done && op_mode_reg[3]) begin
                    // 读取C矩阵（如果需要加C）
                    m_axi_read_arvalid <= 1'b1;
                    m_axi_read_araddr <= input_c_addr_reg + (fetch_index * ACCUM_WIDTH / 8);
                    
                    // 计算剩余读取量，并确保不超过最大突发长度
                    
                    remaining_beats = num_beats_C - fetch_index / ACCUM_ELEMENTS_PER_AXI;
                    
                    if (remaining_beats > MAX_BURST_LENGTH)
                        m_axi_read_arlen <= MAX_BURST_LENGTH;
                    else
                        m_axi_read_arlen <= remaining_beats - 1;
                    
                    current_read_addr <= input_c_addr_reg;
                end
            end else if (m_axi_read_arvalid && m_axi_read_arready) begin
                // 地址已被接受
                m_axi_read_arvalid <= 1'b0;
                axi_read_active <= 1'b1;
                axi_read_expected_beats <= m_axi_read_arlen + 1; // 预期的数据拍数
                axi_read_beat_count <= '0;
            end
            
            // 完成整个突发传输后，重置标志，准备下一次请求
            if (m_axi_read_rvalid && m_axi_read_rready && m_axi_read_rlast) begin
                axi_read_active <= 1'b0;
            end
        end
    end

    // AXI 读取数据通道控制逻辑
    assign m_axi_read_rready = 1'b1; // 总是准备好接收数据
    
    // 从AXI读取数据写入内部缓冲区
    always_ff @(posedge aclk) begin
        if (reset_l) begin
            fetch_index <= '0;
            axi_read_beat_count <= '0;
        end else begin
            // 处理接收的数据
            if (m_axi_read_rvalid && m_axi_read_rready) begin
                axi_read_beat_count <= axi_read_beat_count + 1;
                
                // 根据当前状态，将数据写入相应的缓冲区
                if (current_state == FETCH_A) begin
                    // 写入A缓冲区
                    for (int i = 0; i < PE_ELEMENTS_PER_AXI; i++) begin
                        if (fetch_index + i < INPUT_BUF_DEPTH) begin
                            input_buffer_A[fetch_index + i] <= m_axi_read_rdata[i*PE_DATA_WIDTH+:PE_DATA_WIDTH];
                        end
                    end
                    fetch_index <= fetch_index + PE_ELEMENTS_PER_AXI;
                end else if (current_state == FETCH_B) begin
                    // 写入B缓冲区
                    for (int i = 0; i < PE_ELEMENTS_PER_AXI; i++) begin
                        if (fetch_index + i < INPUT_BUF_DEPTH) begin
                            input_buffer_B[fetch_index + i] <= m_axi_read_rdata[i*PE_DATA_WIDTH+:PE_DATA_WIDTH];
                        end
                    end
                    fetch_index <= fetch_index + PE_ELEMENTS_PER_AXI;
                end else if (current_state == FETCH_C) begin
                    // 写入C缓冲区 (处理不同位宽)
                    for (int i = 0; i < ACCUM_ELEMENTS_PER_AXI; i++) begin
                        if (fetch_index + i < INPUT_BUF_DEPTH) begin
                            input_buffer_C[fetch_index + i] <= m_axi_read_rdata[i*ACCUM_WIDTH+:ACCUM_WIDTH];
                        end
                    end
                    fetch_index <= fetch_index + ACCUM_ELEMENTS_PER_AXI;
                end
                
                // 处理突发传输结束
                if (m_axi_read_rlast) begin
                    axi_read_beat_count <= '0;
                end
            end
            
            // 状态转换时重置索引
            if (next_state != current_state) begin
                if (next_state == FETCH_A || next_state == FETCH_B || next_state == FETCH_C) begin
                    fetch_index <= '0;
                end
            end
        end
    end

    // 获取完成逻辑 - 确保所有数据都已接收
    always_comb begin
        fetch_a_done = (current_state == FETCH_A) && (fetch_index >= num_elements_A) && !axi_read_active;
        fetch_b_done = (current_state == FETCH_B) && (fetch_index >= num_elements_B) && !axi_read_active;
        fetch_c_done = (current_state == FETCH_C) && ((fetch_index >= num_elements_C) || !op_mode_reg[3]) && !axi_read_active;
    end

    // --- 内部缓冲区到计算核心的数据传输 ---
    always_ff @(posedge aclk) begin
        // 从计算核心写入D缓冲区
        if (core_d_write_enable) begin
            if (core_d_addr < OUTPUT_BUF_DEPTH) // 边界检查
                output_buffer_D[core_d_addr] <= core_d_data_out;
        end
    end

    // --- AXI4-Full 主接口 (写 D) ---
    logic m_axi_awvalid_logic, m_axi_wvalid_logic, m_axi_bready_logic;
    logic m_axi_wlast_logic;
    logic [7:0] m_axi_expected_writes;

    assign m_axi_awvalid = m_axi_awvalid_logic;
    assign m_axi_wvalid = m_axi_wvalid_logic;
    assign m_axi_bready = m_axi_bready_logic;
    assign m_axi_wlast = m_axi_wlast_logic;

    // 固定的 AXI 主信号
    assign m_axi_awid = '0;
    assign m_axi_awburst = 2'b01; // INCR
    assign m_axi_awsize = $clog2(AXI_DATA_WIDTH/8);
    assign m_axi_awlock = 1'b0;
    assign m_axi_awcache = 4'b0010; // Normal Non-cacheable Non-bufferable
    assign m_axi_awprot = 3'b000;
    assign m_axi_awqos = 4'b0000;
    assign m_axi_wstrb = {(AXI_DATA_WIDTH/8){1'b1}};

    // AW 通道逻辑
    always_ff @(posedge aclk) begin
        if (reset_l) begin
            m_axi_awvalid_logic <= 1'b0;
            m_axi_awaddr <= '0;
            m_axi_awlen <= '0;
            m_axi_write_active <= 1'b0;
            m_axi_expected_writes <= '0;
        end else begin
            if (current_state == WRITE_BACK_D && !m_axi_write_active && !m_axi_awvalid_logic) begin
                if (num_beats_D_write > 0) begin
                    m_axi_awvalid_logic <= 1'b1;
                    m_axi_awaddr <= output_d_addr_reg;
                    
                    // 计算突发长度 - 确保不超过最大突发长度
                    if (num_beats_D_write > MAX_BURST_LENGTH)
                        m_axi_awlen <= MAX_BURST_LENGTH;
                    else
                        m_axi_awlen <= num_beats_D_write - 1;
                    
                    m_axi_expected_writes <= (num_beats_D_write > MAX_BURST_LENGTH) ? MAX_BURST_LENGTH : (num_beats_D_write - 1);
                end
            end else if (m_axi_awvalid_logic && m_axi_awready) begin
                m_axi_awvalid_logic <= 1'b0;
                m_axi_write_active <= 1'b1;
            end
            
            if (m_axi_bvalid && m_axi_bready_logic) begin
                m_axi_write_active <= 1'b0;
            end
        end
    end

    // W 通道逻辑
    always_ff @(posedge aclk) begin
        if (reset_l) begin
            m_axi_wvalid_logic <= 1'b0;
            m_axi_wlast_logic <= 1'b0;
            m_axi_write_beat_count <= '0;
            write_index <= '0;
            m_axi_wdata <= '0;
        end else begin
            if (m_axi_write_active && m_axi_wready && !m_axi_wvalid_logic && (m_axi_write_beat_count <= m_axi_expected_writes)) begin
                m_axi_wvalid_logic <= 1'b1;
                
                // 从输出缓冲区打包数据
                for (int i = 0; i < ACCUM_ELEMENTS_PER_AXI; i++) begin
                    if (write_index + i < OUTPUT_BUF_DEPTH) begin
                        m_axi_wdata[i*ACCUM_WIDTH+:ACCUM_WIDTH] <= output_buffer_D[write_index + i];
                    end else begin
                        m_axi_wdata[i*ACCUM_WIDTH+:ACCUM_WIDTH] <= '0;
                    end
                end
                
                if (m_axi_write_beat_count == m_axi_expected_writes) begin
                    m_axi_wlast_logic <= 1'b1;
                end else begin
                    m_axi_wlast_logic <= 1'b0;
                end
                
                m_axi_write_beat_count <= m_axi_write_beat_count + 1;
                write_index <= write_index + ACCUM_ELEMENTS_PER_AXI;
            end else if (m_axi_wvalid_logic && m_axi_wready) begin
                m_axi_wvalid_logic <= 1'b0;
            end
            
            if (m_axi_wlast_logic && m_axi_wvalid_logic && m_axi_wready) begin
                m_axi_wlast_logic <= 1'b0;
            end
            
            if (next_state != current_state) begin
                if (next_state == WRITE_BACK_D) begin
                    write_index <= '0;
                    m_axi_write_beat_count <= '0;
                end
            end
        end
    end

    // B 通道逻辑
    assign m_axi_bready_logic = 1'b1; // 总是准备好接收写响应
    
    // 写完成逻辑
    assign write_d_done = m_axi_bvalid && m_axi_bready_logic && (write_index >= num_elements_D);

    // --- 主状态机逻辑 ---
    // 状态寄存器
    always_ff @(posedge aclk) begin
        if (reset_l || control_reg_reset) begin
            current_state <= IDLE;
        end else begin
            current_state <= next_state;
        end
    end

    // 下一状态逻辑
    always_comb begin
        next_state = current_state;
        compute_start_i = 1'b0;

        case (current_state)
            IDLE: begin
                if (control_reg_start) begin
                    next_state = FETCH_A;
                end
            end
            
            FETCH_A: begin
                if (fetch_a_done) begin
                    next_state = FETCH_B;
                end
            end
            
            FETCH_B: begin
                if (fetch_b_done) begin
                    if (op_mode_reg[3] && num_elements_C > 0) begin
                        next_state = FETCH_C;
                    end else begin
                        next_state = COMPUTE;
                    end
                end
            end
            
            FETCH_C: begin
                if (fetch_c_done) begin
                    next_state = COMPUTE;
                end
            end
            
            COMPUTE: begin
                compute_start_i = 1'b1;
                if (compute_core_done) begin
                    compute_start_i = 1'b0;
                    next_state = WRITE_BACK_D;
                end
            end
            
            WRITE_BACK_D: begin
                if (write_d_done) begin
                    next_state = DONE;
                end
            end
            
            DONE: begin
                if (!control_reg_start) begin
                    next_state = IDLE;
                end
            end
            
            default: next_state = IDLE;
        endcase
        
        if (control_reg_reset) begin
            next_state = IDLE;
        end
    end

    // 状态寄存器 DONE 标志
    always_ff @(posedge aclk) begin
        if (reset_l || control_reg_reset) begin
            status_reg_done <= 1'b0;
        end else if (current_state == DONE) begin
            status_reg_done <= 1'b1;
        end else if (current_state != DONE && next_state != DONE) begin
            status_reg_done <= 1'b0;
        end
    end

endmodule