module tensor_compute_core #(
    parameter integer NUM_PE            = 256,       // 处理单元 (PE) 的数量
	parameter integer PE_ARRAY_HEIGHT   = 16,        // PE阵列高度
	parameter integer PE_ARRAY_WIDTH    = 16,        // PE阵列宽度
    parameter integer PE_DATA_WIDTH     = 32,       // PE 数据宽度
    parameter integer ACCUM_WIDTH       = 32,       // 累加器宽度
    parameter integer BUF_ADDR_WIDTH    = 10,       // 缓冲区地址宽度
    parameter integer MATRIX_SIZE_WIDTH = 32,       // 矩阵尺寸编码宽度
    parameter integer K_BITS            = 10        // K 维度位数
) (
    input                                   clk,                  // 时钟
    input                                   reset,                // 复位
    input                                   start_computation_pulse, // 开始计算脉冲信号
    input  [MATRIX_SIZE_WIDTH-1:0]          matrix_size_i,        // 包含 M,N,K 维度信息
    input  [1:0]                            operation_mode_i,     // 位 0: 是否加 C 矩阵 (add_c_matrix)
    input  [2:0]                            precision_mode_i,     // 精度模式:
                                                                  // 000: INT4乘法+INT8累加
                                                                  // 001: INT8乘法+INT16累加
                                                                  // 010: FP16乘法+FP16累加
                                                                  // 011: FP16乘法+FP32累加 (混合精度)
                                                                  // 100: FP32乘法+FP32累加
    
    // 内部缓冲区访问的内存接口
    output logic [BUF_ADDR_WIDTH-1:0]       a_addr_o,             // A 矩阵读取地址
    output logic [BUF_ADDR_WIDTH-1:0]       b_addr_o,             // B 矩阵读取地址
    output logic [BUF_ADDR_WIDTH-1:0]       c_addr_o,             // C 矩阵读取地址
    input  [PE_DATA_WIDTH-1:0]              a_data_i,             // 来自缓冲区的 A 数据
    input  [PE_DATA_WIDTH-1:0]              b_data_i,             // 来自缓冲区的 B 数据
    input  [ACCUM_WIDTH-1:0]                c_data_i,             // 来自缓冲区的 C 数据
    
    // 输出结果到 D 缓冲区
    output logic [BUF_ADDR_WIDTH-1:0]       d_addr_o,             // D 写入地址
    output logic [ACCUM_WIDTH-1:0]          d_data_o,             // 要写入的 D 数据
    output logic                            d_write_enable_o,     // D 写使能
    
    // 状态
    output logic                            computation_done      // 计算完成标志
);
       // 添加二维PE映射所需的信号
    logic [BUF_ADDR_WIDTH-1:0] pe_a_addr[NUM_PE-1:0];  // 每个PE的A矩阵地址
    logic [BUF_ADDR_WIDTH-1:0] pe_b_addr[NUM_PE-1:0];  // 每个PE的B矩阵地址
    logic [BUF_ADDR_WIDTH-1:0] pe_c_addr[NUM_PE-1:0];  // 每个PE的C矩阵地址
    logic [BUF_ADDR_WIDTH-1:0] pe_d_addr[NUM_PE-1:0];  // 每个PE的D矩阵地址
	
	    // PE位置/映射关系
    logic [$clog2(PE_ARRAY_HEIGHT)-1:0]  pe_row[NUM_PE-1:0]; // 每个PE的行索引
    logic [$clog2(PE_ARRAY_WIDTH )-1:0]  pe_col[NUM_PE-1:0]; // 每个PE的列索引
	
	  // 初始化PE索引映射
    integer pe_idx;
    initial begin
        for (pe_idx = 0; pe_idx < NUM_PE; pe_idx = pe_idx + 1) begin
            pe_row[pe_idx] = pe_idx / PE_ARRAY_WIDTH;
            pe_col[pe_idx] = pe_idx % PE_ARRAY_WIDTH;
        end
    end

    // --- 提取矩阵维度 ---
    localparam M_BITS = 11;  // 矩阵 M 维度位数
    localparam N_BITS = 11;  // 矩阵 N 维度位数
    
    logic [M_BITS-1:0] m_dim; // M 维度 (矩阵行数)
    logic [N_BITS-1:0] n_dim; // N 维度 (矩阵列数)
    logic [K_BITS-1:0] k_dim; // K 维度 (内积长度)
    
    // 从矩阵尺寸寄存器提取维度
    assign m_dim = matrix_size_i[MATRIX_SIZE_WIDTH-1 : MATRIX_SIZE_WIDTH-M_BITS];
    assign n_dim = matrix_size_i[MATRIX_SIZE_WIDTH-M_BITS-1 : MATRIX_SIZE_WIDTH-M_BITS-N_BITS];
    assign k_dim = matrix_size_i[K_BITS-1 : 0];
    
    // 加 C 矩阵控制
    logic add_c_matrix = operation_mode_i[0];

    // --- FP32 加法函数 (用于FP32和混合精度FP16/FP32模式下的C加法) ---
    function logic [31:0] fp32_add (input logic [31:0] a, input logic [31:0] b);
        logic sign_a, sign_b, sign_res;        // 符号位
        logic [7:0] exp_a, exp_b, exp_res;     // 指数
        logic [22:0] mant_a, mant_b;           // 尾数
        logic [24:0] mant_a_full, mant_b_full; // 带隐藏位和保护位的尾数
        logic [24:0] mant_res;                 // 结果尾数 (带隐藏位和保护位)
        logic [31:0] result;                   // 最终结果
        int exp_diff;                          // 指数差
		logic [22:0] mant_res_rounded;
        
        // 提取组件
        sign_a = a[31];
        exp_a = a[30:23];
        mant_a = a[22:0];
        sign_b = b[31];
        exp_b = b[30:23];
        mant_b = b[22:0];
        
        // 特殊情况处理
        // 如果有一个操作数是零
        if (exp_a == 8'b0 && mant_a == 23'b0) return b;
        if (exp_b == 8'b0 && mant_b == 23'b0) return a;
        
        // 无穷大和NaN处理
        if (exp_a == 8'hFF) begin
            if (mant_a != 0) return a; // NaN传播
            if (exp_b == 8'hFF && mant_b == 0 && sign_a != sign_b)
                return 32'h7FC00000; // +Inf + (-Inf) = NaN
            return a; // Inf + 任何数 = Inf
        end
        if (exp_b == 8'hFF) return b; // 同上
        
        // 为计算准备完整尾数
        mant_a_full = (exp_a == 0) ? {1'b0, mant_a, 1'b0} : {1'b1, mant_a, 1'b0}; // 非规格化数没有隐藏位1
        mant_b_full = (exp_b == 0) ? {1'b0, mant_b, 1'b0} : {1'b1, mant_b, 1'b0};
        
        // 对齐指数（调整较小的操作数）
        if (exp_a > exp_b) begin
            exp_diff = exp_a - exp_b;
            exp_res = exp_a;
            if (exp_diff > 25) exp_diff = 25; // 限制移位量（超过这个移位就没有精度影响）
            mant_b_full = mant_b_full >> exp_diff;
        end else begin
            exp_diff = exp_b - exp_a;
            exp_res = exp_b;
            if (exp_diff > 25) exp_diff = 25;
            mant_a_full = mant_a_full >> exp_diff;
        end
        
        // 根据符号执行加减
        if (sign_a == sign_b) begin
            mant_res = mant_a_full + mant_b_full;
            sign_res = sign_a;
        end else begin
            // 减法，确保较大的数减较小的数
            if (mant_a_full >= mant_b_full) begin
                mant_res = mant_a_full - mant_b_full;
                sign_res = sign_a;
            end else begin
                mant_res = mant_b_full - mant_a_full;
                sign_res = sign_b;
            end
        end
        
        // 结果规格化
        // 处理进位
        if (mant_res[24]) begin // 需要右移一位
            mant_res = mant_res >> 1;
            exp_res = exp_res + 1;
        end else if (mant_res[23] == 0) begin
            // 处理前导零并左移
            int leading_zeros = 0;
            for (int i = 23; i >= 0; i--) begin
                if (mant_res[i]) break;
                leading_zeros = leading_zeros + 1;
            end
            
            if (leading_zeros == 24) begin
                // 结果为零
                return {sign_res, 31'b0};
            end else if (leading_zeros > 0 && leading_zeros <= exp_res) begin
                mant_res = mant_res << leading_zeros;
                exp_res = exp_res - leading_zeros;
            end else if (leading_zeros > exp_res) begin
                // 结果下溢为非规格化数或零
                mant_res = mant_res << exp_res;
                exp_res = 0;
            end
        end
        
        // 检查溢出
        if (exp_res >= 255) begin
            return {sign_res, 8'hFF, 23'b0}; // 溢出为无穷大
        end
        
        // 简单舍入：向零截断
         mant_res_rounded = mant_res[23:1]; // 最低位中丢弃保护位
        
        // 组装结果
        // 如果尾数全为零，则返回零
        if (mant_res == 0)
            return {sign_res, 31'b0};
            
        result = {sign_res, exp_res[7:0], mant_res_rounded};
        return result;
    endfunction

    // --- FP16 加法函数 ---
    function logic [15:0] fp16_add (input logic [15:0] a, input logic [15:0] b);
        logic sign_a, sign_b, sign_res;        // 符号位
        logic [4:0] exp_a, exp_b, exp_res;     // 指数
        logic [9:0] mant_a, mant_b;            // 尾数
        logic [11:0] mant_a_full, mant_b_full; // 带隐藏位和保护位的尾数
        logic [11:0] mant_res;                 // 结果尾数 (带隐藏位和保护位)
        logic [15:0] result;                   // 最终结果
        int exp_diff;                          // 指数差
		logic [9:0] mant_res_rounded;
        
        // 提取组件
        sign_a = a[15];
        exp_a = a[14:10];
        mant_a = a[9:0];
        sign_b = b[15];
        exp_b = b[14:10];
        mant_b = b[9:0];
        
        // 特殊情况处理
        // 如果有一个操作数是零
        if (exp_a == 5'b0 && mant_a == 10'b0) return b;
        if (exp_b == 5'b0 && mant_b == 10'b0) return a;
        
        // 无穷大和NaN处理
        if (exp_a == 5'h1F) begin
            if (mant_a != 0) return a; // NaN传播
            if (exp_b == 5'h1F && mant_b == 0 && sign_a != sign_b)
                return 16'h7E00; // +Inf + (-Inf) = NaN
            return a; // Inf + 任何数 = Inf
        end
        if (exp_b == 5'h1F) return b; // 同上
        
        // 为计算准备完整尾数
        mant_a_full = (exp_a == 0) ? {1'b0, mant_a, 1'b0} : {1'b1, mant_a, 1'b0}; // 非规格化数没有隐藏位1
        mant_b_full = (exp_b == 0) ? {1'b0, mant_b, 1'b0} : {1'b1, mant_b, 1'b0};
        
        // 对齐指数
        if (exp_a > exp_b) begin
            exp_diff = exp_a - exp_b;
            exp_res = exp_a;
            if (exp_diff > 12) exp_diff = 12; // 限制移位量
            mant_b_full = mant_b_full >> exp_diff;
        end else begin
            exp_diff = exp_b - exp_a;
            exp_res = exp_b;
            if (exp_diff > 12) exp_diff = 12;
            mant_a_full = mant_a_full >> exp_diff;
        end
        
        // 执行加减
        if (sign_a == sign_b) begin
            mant_res = mant_a_full + mant_b_full;
            sign_res = sign_a;
        end else begin
            if (mant_a_full >= mant_b_full) begin
                mant_res = mant_a_full - mant_b_full;
                sign_res = sign_a;
            end else begin
                mant_res = mant_b_full - mant_a_full;
                sign_res = sign_b;
            end
        end
        
        // 规格化结果
        if (mant_res[11]) begin // 需要右移
            mant_res = mant_res >> 1;
            exp_res = exp_res + 1;
        end else if (mant_res[10] == 0) begin
            // 处理前导零
            int leading_zeros = 0;
            for (int i = 10; i >= 0; i--) begin
                if (mant_res[i]) break;
                leading_zeros = leading_zeros + 1;
            end
            
            if (leading_zeros == 11) begin
                // 结果为零
                return {sign_res, 15'b0};
            end else if (leading_zeros > 0 && leading_zeros <= exp_res) begin
                mant_res = mant_res << leading_zeros;
                exp_res = exp_res - leading_zeros;
            end else if (leading_zeros > exp_res) begin
                // 下溢
                mant_res = mant_res << exp_res;
                exp_res = 0;
            end
        end
        
        // 溢出检查
        if (exp_res >= 31) begin
            return {sign_res, 5'h1F, 10'b0}; // 溢出为无穷大
        end
        
        // 简单舍入
         mant_res_rounded = mant_res[10:1];
        
        // 组装结果
        if (mant_res == 0)
            return {sign_res, 15'b0};
            
        result = {sign_res, exp_res[4:0], mant_res_rounded};
        return result;
    endfunction

    // --- 状态机 ---
     typedef enum logic [4:0] {
        IDLE,              // 空闲状态，等待启动
        INIT_COMPUTE,      // 初始化计算参数
        FETCH_OPERANDS,    // 设置地址准备获取数据
        FETCH_PE_DATA,     // 逐个获取PE的A、B数据
        COMPUTE_STEP,      // 启动计算
        WAIT_PE_DONE,      // 等待PE完成当前计算步骤
        LOAD_C,            // 设置C矩阵地址
        LOAD_PE_C,         // 逐个加载PE的C数据
        ADD_C,             // 将C加到计算结果上
        WRITE_RESULT,      // 设置结果写回地址
        WRITE_PE_RESULT,   // 逐个写回PE结果
        UPDATE_INDICES,    // 更新索引，准备下一轮计算
        DONE               // 计算完成
    } core_state_t;
    core_state_t current_state, next_state;
    logic [$clog2(NUM_PE):0] next_fetch_pe;  // 下一个获取数据的PE索引
    logic [$clog2(NUM_PE):0] current_pe_idx; // 当前处理的PE索引
	logic data_ready;                           // 数据就绪标志
    

    // --- 计数器和索引 ---
    logic [M_BITS-1:0] m_index;    // M 维度当前处理索引
    logic [N_BITS-1:0] n_index;    // N 维度当前处理索引
    logic [K_BITS-1:0] k_index;    // K 维度当前处理索引
    
    // --- PE 控制信号 ---
    logic [NUM_PE-1:0] pe_start;         // PEs 的启动信号
    logic pe_first_step;                 // PEs 的第一步信号
    logic [NUM_PE-1:0] pe_done;          // 来自 PEs 的完成信号
    
    // --- PE 操作数和结果 ---
    logic [ACCUM_WIDTH-1:0] pe_results[NUM_PE-1:0]; // 来自 PEs 的结果
    
    // --- 活动 PE 跟踪 ---
    logic [NUM_PE-1:0] pe_active;        // 活动 PE 掩码
    logic [$clog2(NUM_PE)+1:0] active_pe_count; // 活动 PE 数量
    
    // --- 结果缓冲区 ---
    logic [ACCUM_WIDTH-1:0] result_buffer[NUM_PE-1:0]; // 存储中间结果的缓冲区

    // --- 状态寄存器 ---
    always_ff @(posedge clk or posedge reset) begin
        if (reset)
            current_state <= IDLE;
        else
            current_state <= next_state;
    end
    
    // --- 下一状态逻辑 ---
   always_comb begin
        next_state = current_state;
        
        case (current_state)
            IDLE: begin
                if (start_computation_pulse)
                    next_state = INIT_COMPUTE;
            end
            
            INIT_COMPUTE: begin
                next_state = FETCH_OPERANDS;
            end
            
            FETCH_OPERANDS: begin
                next_state = FETCH_PE_DATA;
            end
            
            FETCH_PE_DATA: begin
                if (current_pe_idx >= active_pe_count - 1 && data_ready)
                    next_state = COMPUTE_STEP;
            end
            
            COMPUTE_STEP: begin
                next_state = WAIT_PE_DONE;
            end
            
            WAIT_PE_DONE: begin
                // 检查PE是否完成
                logic all_active_pes_done = 1'b1;
                for (int i = 0; i < NUM_PE; i++) begin
                    if (pe_active[i] && !pe_done[i]) begin
                        all_active_pes_done = 1'b0;
                        break;
                    end
                end
                
                if (all_active_pes_done) begin
                    if (k_index == k_dim - 1) begin
                        // 所有K步骤完成
                        if (add_c_matrix)
                            next_state = LOAD_C;
                        else
                            next_state = WRITE_RESULT;
                    end else begin
                        // 继续下一个K步骤
                        next_state = FETCH_OPERANDS;
                    end
                end
            end
            
            LOAD_C: begin
                next_state = LOAD_PE_C;
            end
            
            LOAD_PE_C: begin
                if (current_pe_idx >= active_pe_count - 1 && data_ready)
                    next_state = ADD_C;
            end
            
            ADD_C: begin
                next_state = WRITE_RESULT;
            end
            
            WRITE_RESULT: begin
                next_state = WRITE_PE_RESULT;
            end
            
            WRITE_PE_RESULT: begin
                if (current_pe_idx >= active_pe_count - 1)
                    next_state = UPDATE_INDICES;
            end
            
            UPDATE_INDICES: begin
                if (m_index == m_dim - 1 && n_index == n_dim - 1)
                    next_state = DONE;
                else
                    next_state = INIT_COMPUTE;
            end
            
            DONE: begin
                next_state = IDLE;
            end
            
            default: next_state = IDLE;
        endcase
    end
    
    // --- 控制逻辑和数据路径 ---
      always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            // 复位所有状态
            m_index <= 0;
            n_index <= 0;
            k_index <= 0;
            pe_first_step <= 0;
            d_write_enable_o <= 0;
            computation_done <= 0;
            a_addr_o <= 0;
            b_addr_o <= 0;
            c_addr_o <= 0;
            d_addr_o <= 0;
            d_data_o <= 0;
            current_pe_idx <= 0;
            active_pe_count <= 0;
            data_ready <= 0;
            
            for (int i = 0; i < NUM_PE; i++) begin
                pe_start[i] <= 0;
                pe_active[i] <= 0;
                result_buffer[i] <= 0;
            end
        end else begin
            // 默认值
            pe_first_step <= 0;
            d_write_enable_o <= 0;
            computation_done <= 0;
            data_ready <= 0;
            
            for (int i = 0; i < NUM_PE; i++) begin
                pe_start[i] <= 0; // 默认不启动PE
            end
            
            case (current_state)
                IDLE: begin
                    // 重置状态
                    m_index <= 0;
                    n_index <= 0;
                    k_index <= 0;
                    current_pe_idx <= 0;
                    active_pe_count <= 0;
                    computation_done <= 0;
                    
                    for (int i = 0; i < NUM_PE; i++) begin
                        pe_active[i] <= 0;
                    end
                end
                
                INIT_COMPUTE: begin
                    k_index <= 0; // 重置K索引
                    current_pe_idx <= 0;
                    active_pe_count <= 0;
                    
                    // 激活所有需要的PE
                    for (int i = 0; i < NUM_PE; i++) begin
                        if (pe_row[i] < m_dim && pe_col[i] < n_dim) begin
                            // 只在矩阵范围内激活PE
                            pe_active[i] <= 1;
                            active_pe_count <= active_pe_count + 1;
                        end else begin
                            pe_active[i] <= 0;
                        end
                    end
                end
                
                FETCH_OPERANDS: begin
                    // 设置所有PE的地址
                    for (int i = 0; i < NUM_PE; i++) begin
                        if (pe_active[i]) begin
                            // 输出固定数据流模式
                            pe_a_addr[i] <= pe_row[i] * k_dim + k_index;  // A[m][k]
                            pe_b_addr[i] <= k_index * n_dim + pe_col[i];  // B[k][n]
                            pe_c_addr[i] <= pe_row[i] * n_dim + pe_col[i]; // C[m][n]
                            pe_d_addr[i] <= pe_row[i] * n_dim + pe_col[i]; // D[m][n]
                        end
                    end
                    current_pe_idx <= 0; // 重置PE索引准备获取数据
                end
                
                FETCH_PE_DATA: begin
                    // 逐个为PE获取数据
                    if (pe_active[current_pe_idx]) begin
                        a_addr_o <= pe_a_addr[current_pe_idx];
                        b_addr_o <= pe_b_addr[current_pe_idx];
                        // 这里假设数据可以在一个周期内获取，实际上可能需要等待数据就绪
                        data_ready <= 1;
                    end
                    
                    if (data_ready) begin
                        if (current_pe_idx < active_pe_count - 1) begin
                            current_pe_idx <= current_pe_idx + 1;
                            data_ready <= 0;
                        end
                    end
                end
                
                COMPUTE_STEP: begin
                    // 设置第一步标志
                    if (k_index == 0)
                        pe_first_step <= 1;
                    
                    // 启动所有活跃PE进行计算
                    for (int i = 0; i < NUM_PE; i++) begin
                        if (pe_active[i])
                            pe_start[i] <= 1;
                    end
                end
                
                WAIT_PE_DONE: begin
                    // 等待所有活动PE完成 - 无需额外操作
                    // 状态转换逻辑在always_comb块中
                    
                    // 如果这是最后一个K步骤，保存PE结果
                    if (k_index == k_dim - 1) begin
                        for (int i = 0; i < NUM_PE; i++) begin
                            if (pe_active[i] && pe_done[i])
                                result_buffer[i] <= pe_results[i];
                        end
                    end
                    
                    // 更新K索引为下一步
                    if (next_state == FETCH_OPERANDS) begin
                        k_index <= k_index + 1;
                    end
                end
                
                LOAD_C: begin
                    // 设置准备加载C矩阵
                    current_pe_idx <= 0;
                    data_ready <= 0;
                end
                
                LOAD_PE_C: begin
                    // 逐个为PE加载C数据
                    if (pe_active[current_pe_idx]) begin
                        c_addr_o <= pe_c_addr[current_pe_idx];
                        data_ready <= 1;
                    end
                    
                    if (data_ready) begin
                        if (current_pe_idx < active_pe_count - 1) begin
                            current_pe_idx <= current_pe_idx + 1;
                            data_ready <= 0;
                        end
                    end
                end
                
                
                ADD_C: begin
                    // 将 C 加到结果上
                    // 根据精度模式选择加法逻辑
                    case (precision_mode_i)
                        3'b000: begin // INT4乘法+INT8累加 -> INT8 加法
                            for (int i = 0; i < NUM_PE; i++) begin
                                if (pe_active[i]) begin
                                    // 简单的 8 位有符号加法，需要提取低 8 位
                                    logic signed [7:0] res8 = result_buffer[i][7:0]; 
                                    logic signed [7:0] c8 = c_data_i[7:0];
									logic signed [7:0] sum8;//为了综合方便，下同
									sum8 = res8 + c8;
                                    result_buffer[i][7:0] <= sum8;
                                    // 符号扩展到高位
                                    result_buffer[i][ACCUM_WIDTH-1:8] <= {(ACCUM_WIDTH-8){sum8[7]}}; 
                                end
                            end
                        end
                        3'b001: begin // INT8乘法+INT16累加 -> INT16 加法
                            for (int i = 0; i < NUM_PE; i++) begin
                                if (pe_active[i]) begin
                                    // 简单的 16 位有符号加法
                                    logic signed [15:0] res16 = result_buffer[i][15:0];
                                    logic signed [15:0] c16 = c_data_i[15:0];
									logic signed [15:0] sum16;
									sum16 = res16 + c16;
                                    result_buffer[i][15:0] <= res16 + c16;
                                    // 符号扩展到高位
                                    result_buffer[i][ACCUM_WIDTH-1:16] <= {(ACCUM_WIDTH-16){sum16[15]}};
                                end
                            end
                        end
                        3'b010: begin // FP16乘法+FP16累加 -> FP16 加法
                            for (int i = 0; i < NUM_PE; i++) begin
                                if (pe_active[i]) begin
                                    // 使用 FP16 加法函数
                                    logic [15:0] res_fp16 = result_buffer[i][15:0];
                                    logic [15:0] c_fp16 = c_data_i[15:0];
                                    result_buffer[i][15:0] <= fp16_add(res_fp16, c_fp16);
                                    // 零扩展到高位
                                    result_buffer[i][ACCUM_WIDTH-1:16] <= '0;
                                end
                            end
                        end
                        3'b011, 3'b100: begin // FP16乘法+FP32累加 或 FP32乘法+FP32累加 -> FP32 加法
                            for (int i = 0; i < NUM_PE; i++) begin
                                if (pe_active[i])
                                    result_buffer[i] <= fp32_add(result_buffer[i], c_data_i);
                            end
                        end
                        default: begin
                            // 默认整数加法
                            for (int i = 0; i < NUM_PE; i++) begin
                                if (pe_active[i])
                                    result_buffer[i] <= result_buffer[i] + c_data_i;
                            end
                        end
                    endcase
                end
				
		      	  UPDATE_INDICES: begin
                    // 按列优先遍历矩阵
                    if (n_index < n_dim - 1) begin
                        n_index <= n_index + 1;
                    end else begin
                        n_index <= 0;
                        if (m_index < m_dim - 1)
                            m_index <= m_index + 1;
                        else
                            m_index <= 0;
                    end
                end
                
                DONE: begin
                    computation_done <= 1;
                end
            endcase
        end
    end
    
    // --- 实例化 PEs ---
    genvar i;
    generate
        for (i = 0; i < NUM_PE; i++) begin : pe_inst
            mixed_precision_pe #(
                .PE_DATA_WIDTH(PE_DATA_WIDTH),
                .ACCUM_WIDTH(ACCUM_WIDTH),
                .PE_ID(i),
                .K_BITS(K_BITS)
            ) pe (
                .clk(clk),
                .reset(reset),
                .start(pe_start[i]),
                .first_step(pe_first_step),
                .k_dim_i(k_dim),
                .a_data_i(a_data_i),
                .b_data_i(b_data_i),
				.c_data_i(c_data_i),
                .precision_mode(precision_mode_i),
                .result_out(pe_results[i]),
                .pe_done_o(pe_done[i])
            );
        end
    endgenerate

endmodule