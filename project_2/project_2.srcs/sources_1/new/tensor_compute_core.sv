module tensor_compute_core #(
    parameter integer NUM_PE            = 256,
    parameter integer PE_ARRAY_HEIGHT   = 16,
    parameter integer PE_ARRAY_WIDTH    = 16,
    parameter integer PE_DATA_WIDTH     = 32,
    parameter integer ACCUM_WIDTH       = 32,
    parameter integer BUF_ADDR_WIDTH    = 10,
    parameter integer MATRIX_SIZE_WIDTH = 32,
    parameter integer K_BITS            = 10
) (
    input                                   clk,
    input                                   reset,
    input                                   start_computation_pulse,
    input  [MATRIX_SIZE_WIDTH-1:0]          matrix_size_i,
    input  [1:0]                            operation_mode_i,     // 位 0: 是否加C矩阵
    input  [2:0]                            precision_mode_i,     // 精度模式
    
    output logic [BUF_ADDR_WIDTH-1:0]       a_addr_o,
    output logic [BUF_ADDR_WIDTH-1:0]       b_addr_o,
    output logic [BUF_ADDR_WIDTH-1:0]       c_addr_o,
    input  [PE_DATA_WIDTH-1:0]              a_data_i,
    input  [PE_DATA_WIDTH-1:0]              b_data_i,
    input  [ACCUM_WIDTH-1:0]                c_data_i,
    
    output logic [BUF_ADDR_WIDTH-1:0]       d_addr_o,
    output logic [ACCUM_WIDTH-1:0]          d_data_o,
    output logic                            d_write_enable_o,
    
    output logic                            computation_done
);

    // ===== 提取矩阵维度 =====
    localparam M_BITS = 11;
    localparam N_BITS = 11;
    
    logic [M_BITS-1:0] m_dim;     // 矩阵A的行数
    logic [N_BITS-1:0] n_dim;     // 矩阵B的列数/结果矩阵列数
    logic [K_BITS-1:0] k_dim;     // 内积维度(A的列数/B的行数)
    
    assign m_dim = matrix_size_i[MATRIX_SIZE_WIDTH-1 : MATRIX_SIZE_WIDTH-M_BITS];
    assign n_dim = matrix_size_i[MATRIX_SIZE_WIDTH-M_BITS-1 : MATRIX_SIZE_WIDTH-M_BITS-N_BITS];
    assign k_dim = matrix_size_i[K_BITS-1 : 0];
    
    // ===== 简化后的状态机 =====
    typedef enum logic [3:0] {
        IDLE,               // 空闲状态
        INIT_TILE,          // 初始化当前分块
        SET_PE_ADDR,        // 设置地址
        LOAD_PE_DATA,       // 加载数据
        COMPUTE,            // 执行计算
        STORE_RESULTS,      // 存储结果
        NEXT_TILE,          // 准备下一个分块
        DONE                // 计算完成
    } core_state_t;
    
    core_state_t current_state, next_state;
    
    // ===== Tiling控制变量 =====
    logic [M_BITS-1:0] tile_m_start;     // 当前分块的M维度起始位置
    logic [N_BITS-1:0] tile_n_start;     // 当前分块的N维度起始位置
    logic [M_BITS-1:0] tile_m_end;       // 当前分块的M维度结束位置
    logic [N_BITS-1:0] tile_n_end;       // 当前分块的N维度结束位置
    logic [M_BITS-1:0] tile_m_size;      // 当前分块的M维度大小
    logic [N_BITS-1:0] tile_n_size;      // 当前分块的N维度大小
    
    logic last_m_tile;                   // 是否为M维度的最后一个分块
    logic last_n_tile;                   // 是否为N维度的最后一个分块
    
    // ===== 计算控制变量 =====
    logic [K_BITS-1:0] k_index;                  // 当前处理的K索引
    logic [$clog2(NUM_PE):0] pe_idx;             // 当前操作的PE索引
    logic [NUM_PE-1:0] pe_active;                // 活动PE标志
    logic [$clog2(NUM_PE)+1:0] active_pe_count;  // 当前分块中活动PE的数量
    
    // PE控制信号
    logic [NUM_PE-1:0] pe_start;
    logic [NUM_PE-1:0] pe_load_c;
    logic [NUM_PE-1:0] pe_add_c;
    logic [NUM_PE-1:0] pe_done;
    logic first_k_step;
    
    // 数据传输控制
    logic all_pes_computed;
    logic all_results_stored;
    
    // PE结果和地址
    logic [ACCUM_WIDTH-1:0] pe_results[NUM_PE-1:0];
    logic [BUF_ADDR_WIDTH-1:0] pe_a_addr[NUM_PE-1:0];
    logic [BUF_ADDR_WIDTH-1:0] pe_b_addr[NUM_PE-1:0];
    logic [BUF_ADDR_WIDTH-1:0] pe_c_addr[NUM_PE-1:0];
    logic [BUF_ADDR_WIDTH-1:0] pe_d_addr[NUM_PE-1:0];
    
    // 当前PE的位置映射
    logic [$clog2(PE_ARRAY_HEIGHT)-1:0] pe_row[NUM_PE-1:0];
    logic [$clog2(PE_ARRAY_WIDTH)-1:0] pe_col[NUM_PE-1:0];
    
    // 加C矩阵控制
    logic add_c_matrix;
    assign add_c_matrix = operation_mode_i[0];
    
    // 初始化PE位置映射
    initial begin
        for (int i = 0; i < NUM_PE; i++) begin
            pe_row[i] = i / PE_ARRAY_WIDTH;
            pe_col[i] = i % PE_ARRAY_WIDTH;
        end
    end
    
    // ===== Tiling计算函数 =====
    // 计算每个分块的大小和边界
    function void calculate_tile_bounds();
        // 计算当前分块的边界
        tile_m_end = ((tile_m_start + PE_ARRAY_HEIGHT) > m_dim) ? 
                      m_dim : (tile_m_start + PE_ARRAY_HEIGHT);
        tile_n_end = ((tile_n_start + PE_ARRAY_WIDTH) > n_dim) ? 
                      n_dim : (tile_n_start + PE_ARRAY_WIDTH);
        
        // 计算当前分块的大小
        tile_m_size = tile_m_end - tile_m_start;
        tile_n_size = tile_n_end - tile_n_start;
        
        // 检查是否为最后一个分块
        last_m_tile = (tile_m_end == m_dim);
        last_n_tile = (tile_n_end == n_dim);
    endfunction
    
    // ===== 状态转换逻辑 =====
    always_comb begin
        next_state = current_state;
        
        case (current_state)
            IDLE: begin
                if (start_computation_pulse)
                    next_state = INIT_TILE;
            end
            
            INIT_TILE: begin
                next_state = SET_PE_ADDR;
            end
            
            SET_PE_ADDR: begin
                if (pe_idx >= active_pe_count)
                    next_state = LOAD_PE_DATA;
            end
            
            LOAD_PE_DATA: begin
                if (pe_idx >= active_pe_count)
                    next_state = COMPUTE;
            end
            
            COMPUTE: begin
                if (all_pes_computed) begin
                    if (k_index == k_dim - 1)
                        next_state = STORE_RESULTS;
                    else
                        next_state = SET_PE_ADDR; // 准备下一个K步骤
                end
            end
            
            STORE_RESULTS: begin
                if (all_results_stored)
                    next_state = NEXT_TILE;
            end
            
            NEXT_TILE: begin
                if (last_m_tile && last_n_tile)
                    next_state = DONE;
                else
                    next_state = INIT_TILE;
            end
            
            DONE: begin
                next_state = IDLE;
            end
        endcase
    end
	
	// 计算该PE对应的全局矩阵坐标和缓冲区地址
                            logic [M_BITS-1:0] global_m;
                            logic [N_BITS-1:0] global_n;
    
    // ===== 主控制逻辑 =====
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            // 复位所有状态
            current_state <= IDLE;
            tile_m_start <= '0;
            tile_n_start <= '0;
            k_index <= '0;
            pe_idx <= '0;
            
            computation_done <= 0;
            d_write_enable_o <= 0;
            a_addr_o <= '0;
            b_addr_o <= '0;
            c_addr_o <= '0;
            d_addr_o <= '0;
            d_data_o <= '0;
            
            for (int i = 0; i < NUM_PE; i++) begin
                pe_start[i] <= 0;
                pe_load_c[i] <= 0;
                pe_add_c[i] <= 0;
                pe_active[i] <= 0;
            end
            
            all_pes_computed <= 0;
            all_results_stored <= 0;
            
        end else begin
            current_state <= next_state;
            
            // 默认清除单周期信号，脉冲信号设置
            d_write_enable_o <= 0;
            for (int i = 0; i < NUM_PE; i++) begin
                pe_start[i] <= 0;
                pe_load_c[i] <= 0;
                pe_add_c[i] <= 0;
            end
            
            case (current_state)
                IDLE: begin
                    computation_done <= 0;
                    // 初始化分块坐标
                    tile_m_start <= '0;
                    tile_n_start <= '0;
                    k_index <= '0;
                end
                
                INIT_TILE: begin
                    // 计算当前分块的边界和大小
                    calculate_tile_bounds();
                    
                    // 初始化变量
                    k_index <= '0;
                    pe_idx <= '0;
                    active_pe_count <= '0;
                    
                    // 确定哪些PE是活动的
                    for (int i = 0; i < NUM_PE; i++) begin
                        logic [M_BITS-1:0] local_m;
                        logic [N_BITS-1:0] local_n;
                        
                        local_m = pe_row[i];
                        local_n = pe_col[i];
                        
                        // 如果PE在当前分块范围内则激活
                        if (local_m < tile_m_size && local_n < tile_n_size) begin
                            pe_active[i] <= 1;
                            active_pe_count <= active_pe_count + 1;
                            
                          
                            
                            global_m = tile_m_start + local_m;
                            global_n = tile_n_start + local_n;
                            
                            pe_a_addr[i] <= global_m * k_dim;                // A[global_m][0]起始地址
                            pe_b_addr[i] <= global_n;                        // B[0][global_n]起始地址
                            pe_c_addr[i] <= global_m * n_dim + global_n;     // C[global_m][global_n]
                            pe_d_addr[i] <= global_m * n_dim + global_n;     // D[global_m][global_n]
                        end else begin
                            pe_active[i] <= 0;
                        end
                    end
                    
                    first_k_step <= 1;  // 标记首个K迭代
                    all_pes_computed <= 0;
                    all_results_stored <= 0;
                end
                
                SET_PE_ADDR: begin
                    // 设置当前PE的地址
                    if (pe_idx < active_pe_count) begin
                        int pe_id = 0;
                        logic found = 0;
                        
                        // 找到下一个活动PE
                        for (int i = 0; i < NUM_PE; i++) begin
                            if (pe_active[i]) begin
                                if (pe_id == pe_idx && !found) begin
                                    // 设置地址
                                    a_addr_o <= pe_a_addr[i] + k_index;
                                    b_addr_o <= pe_b_addr[i] + k_index * n_dim;
                                    
                                    if (first_k_step && add_c_matrix) begin
                                        c_addr_o <= pe_c_addr[i];
                                    end
                                    
                                    found = 1;
                                    pe_idx <= pe_idx + 1;
                                    break;
                                end
                                pe_id = pe_id + 1;
                            end
                        end
                    end
                end
                
                LOAD_PE_DATA: begin
                    // 加载数据并启动PE
                    if (pe_idx < active_pe_count) begin
                        int pe_id = 0;
                        logic found = 0;
                        
                        // 找到下一个活动PE
                        for (int i = 0; i < NUM_PE; i++) begin
                            if (pe_active[i]) begin
                                if (pe_id == pe_idx && !found) begin
                                    // 启动PE
                                    pe_start[i] <= 1;
                                    
                                    // 如果是第一个K步骤，加载C数据
                                    if (first_k_step && add_c_matrix) begin
                                        pe_load_c[i] <= 1;
                                    end
                                    
                                    found = 1;
                                    pe_idx <= pe_idx + 1;
                                    break;
                                end
                                pe_id = pe_id + 1;
                            end
                        end
                    end
                end
                
                COMPUTE: begin
                    // 检查所有PE是否完成计算
                    all_pes_computed <= 1;
                    for (int i = 0; i < NUM_PE; i++) begin
                        if (pe_active[i] && !pe_done[i]) begin
                            all_pes_computed <= 0;
                            break;
                        end
                    end
                    
                    // 当所有PE完成时，准备下一步
                    if (all_pes_computed) begin
                        if (k_index < k_dim - 1) begin
                            // 进入下一个K迭代
                            k_index <= k_index + 1;
                            pe_idx <= '0;
                            first_k_step <= 0;
                        end else if (k_index == k_dim - 1) begin
                            // 最后一个K迭代，准备将C加到结果
                            if (add_c_matrix) begin
                                for (int i = 0; i < NUM_PE; i++) begin
                                    if (pe_active[i]) begin
                                        pe_add_c[i] <= 1;
                                    end
                                end
                            end
                            
                            // 重置索引准备存储结果
                            pe_idx <= '0;
                        end
                    end
                end
                
                STORE_RESULTS: begin
                    // 存储计算结果到输出缓冲区
                    if (pe_idx < active_pe_count) begin
                        int pe_id = 0;
                        logic found = 0;
                        
                        // 找到下一个要存储结果的PE
                        for (int i = 0; i < NUM_PE; i++) begin
                            if (pe_active[i]) begin
                                if (pe_id == pe_idx && !found) begin
                                    d_addr_o <= pe_d_addr[i];
                                    d_data_o <= pe_results[i];
                                    d_write_enable_o <= 1;
                                    
                                    found = 1;
                                    pe_idx <= pe_idx + 1;
                                    break;
                                end
                                pe_id = pe_id + 1;
                            end
                        end
                    end else begin
                        // 所有结果已存储
                        all_results_stored <= 1;
                    end
                end
                
                NEXT_TILE: begin
                    // 更新分块索引
                    if (!last_n_tile) begin
                        // 移动到同一行的下一个分块
                        tile_n_start <= tile_n_start + PE_ARRAY_WIDTH;
                    end else if (!last_m_tile) begin
                        // 移动到下一行的第一个分块
                        tile_m_start <= tile_m_start + PE_ARRAY_HEIGHT;
                        tile_n_start <= '0;
                    end
                end
                
                DONE: begin
                    computation_done <= 1;
                end
            endcase
        end
    end
    
    // ===== 实例化PEs =====
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
                .load_c(pe_load_c[i]),
                .add_c_to_result(pe_add_c[i]),
                .first_step(first_k_step),
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