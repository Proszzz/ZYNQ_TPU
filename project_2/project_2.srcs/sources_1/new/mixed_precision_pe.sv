module mixed_precision_pe #(
    parameter integer PE_DATA_WIDTH = 32,        // PE 数据宽度
    parameter integer ACCUM_WIDTH = 32,          // 累加器宽度
    parameter integer PE_ID = 0,                 // PE 编号
    parameter integer K_BITS = 10                // K 维度宽度
) (
    input clk,                                   // 时钟信号
    input reset,                                 // 复位信号

    // 控制信号
    input start,                                 // 开始计算信号
    input load_c,                                // 新增: 加载C值信号
    input add_c_to_result,                       // 新增: 将C加到最终结果
    input first_step,                            // 第一个K步骤标志
    input [K_BITS-1:0] k_dim_i,                  // K维度大小

    // 数据输入
    input [PE_DATA_WIDTH-1:0] a_data_i,          // A矩阵数据
    input [PE_DATA_WIDTH-1:0] b_data_i,          // B矩阵数据
    input [ACCUM_WIDTH-1:0] c_data_i,            // C矩阵数据
    
    // 精度模式
    input [2:0] precision_mode,                  // 精度模式选择
    
    // 输出信号
    output logic [ACCUM_WIDTH-1:0] result_out,   // 计算结果
    output logic pe_done_o                       // 完成信号
);

    // 添加综合指示，禁用DSP资源（可选）
    (* use_dsp = "no" *)

    // --- 简化后的状态机 ---
    typedef enum logic [2:0] {
        PE_IDLE,            // 空闲状态
        PE_MULTIPLY,        // 执行乘法
        PE_ACCUMULATE,      // 执行累加
        PE_ADD_C,           // 加上C矩阵数据
        PE_DONE             // 完成状态
    } pe_state_t;
    
    pe_state_t current_state, next_state;
    
    // --- 内部寄存器 ---
    logic [PE_DATA_WIDTH-1:0] a_operand, b_operand;     // 输入操作数
    logic [ACCUM_WIDTH-1:0] c_value;                    // 存储C矩阵值
    logic [ACCUM_WIDTH-1:0] mul_result;                 // 乘法结果
    logic [K_BITS-1:0] k_step_count;                    // K步骤计数器
    
    // 不同精度的累加器
    logic signed [7:0] accum_int8;                      // INT8累加器
    logic signed [15:0] accum_int16;                    // INT16累加器
    logic [15:0] accum_fp16;                            // FP16累加器
    logic [31:0] accum_fp32;                            // FP32累加器
    
    // 乘法器中间变量
    logic [47:0] partial_product;                       // 乘法部分积
    logic [5:0] mul_counter;                            // 乘法迭代计数器
    logic [5:0] max_mul_iterations;                     // 最大迭代次数
    
    // 浮点乘法辅助变量
    logic mul_sign;                                     // 乘法结果符号位
    logic [7:0] exp_result;                             // 指数结果
    logic [3:0] leading_zeros;                          // 前导零计数
    
    // 提取的操作数变量
    logic signed [7:0] a_int4, b_int4;                  // INT4操作数
    logic signed [15:0] a_int8, b_int8;                  // INT8操作数
    logic [15:0] a_fp16, b_fp16;                        // FP16操作数
    logic [31:0] a_fp32, b_fp32;                        // FP32操作数
    
    // 浮点格式分量
    logic a_sign, b_sign;                               // 符号位
    logic [7:0] a_exp, b_exp;                           // 指数
    logic [22:0] a_mant, b_mant;                        // 尾数
    logic [23:0] a_mant_full, b_mant_full;              // 带隐藏位的尾数
	
	logic fp_special_case;       // 特殊情况标志
    logic [31:0] special_result; // 特殊情况结果
    
    // 操作数提取逻辑
    always_comb begin
        // 整数操作数提取
        a_int4 = {{5{a_operand[3]}},a_operand[2:0]};
        b_int4 = {{5{b_operand[3]}},b_operand[2:0]};
        a_int8 = {{9{a_operand[7]}},a_operand[6:0]};
        b_int8 = {{9{b_operand[7]}},b_operand[6:0]};
        
        // 浮点操作数提取
        a_fp16 = a_operand[15:0];
        b_fp16 = b_operand[15:0];
        a_fp32 = a_operand;
        b_fp32 = b_operand;
        
        // FP16分量提取
        if (precision_mode == 3'b010 || precision_mode == 3'b011) begin
            a_sign = a_fp16[15];
            a_exp = {3'b0, a_fp16[14:10]};
            a_mant = {13'b0, a_fp16[9:0]};
            a_mant_full = (a_exp[4:0] == 5'b0) ? {1'b0, a_mant[9:0], 13'b0} : {1'b1, a_mant[9:0], 13'b0};
            
            b_sign = b_fp16[15];
            b_exp = {3'b0, b_fp16[14:10]};
            b_mant = {13'b0, b_fp16[9:0]};
            b_mant_full = (b_exp[4:0] == 5'b0) ? {1'b0, b_mant[9:0], 13'b0} : {1'b1, b_mant[9:0], 13'b0};
        end
        // FP32分量提取
        else if (precision_mode == 3'b100) begin
            a_sign = a_fp32[31];
            a_exp = a_fp32[30:23];
            a_mant = a_fp32[22:0];
            a_mant_full = (a_exp == 8'b0) ? {1'b0, a_mant} : {1'b1, a_mant};
            
            b_sign = b_fp32[31];
            b_exp = b_fp32[30:23];
            b_mant = b_fp32[22:0];
            b_mant_full = (b_exp == 8'b0) ? {1'b0, b_mant} : {1'b1, b_mant};
        end
        else begin
            // 默认值
            a_sign = 1'b0;
            a_exp = 8'b0;
            a_mant = 23'b0;
            a_mant_full = 24'b0;
            
            b_sign = 1'b0;
            b_exp = 8'b0;
            b_mant = 23'b0;
            b_mant_full = 24'b0;
        end
    end
    
    // 状态寄存器更新
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            current_state <= PE_IDLE;
            mul_counter <= 0;
        end else begin
            current_state <= next_state;
            
            // 乘法迭代计数器控制
            if (current_state == PE_IDLE || next_state == PE_IDLE) begin
                mul_counter <= 0;
            end else if (current_state == PE_MULTIPLY && mul_counter < max_mul_iterations) begin
                mul_counter <= mul_counter + 1;
            end
        end
    end
    
    // 下一状态逻辑
    always_comb begin
        next_state = current_state;
        pe_done_o = 1'b0;
        
        case (current_state)
            PE_IDLE: begin
                if (start)
                    next_state = PE_MULTIPLY;
            end
            
            PE_MULTIPLY: begin
                // 当乘法迭代完成时进入累加状态
                case (precision_mode)
                    3'b000: if (mul_counter >= 4) next_state = PE_ACCUMULATE; // INT4
                    3'b001: if (mul_counter >= 8) next_state = PE_ACCUMULATE; // INT8
                    3'b010, 3'b011: if (mul_counter >= 11) next_state = PE_ACCUMULATE; // FP16
                    3'b100: if (mul_counter >= 24) next_state = PE_ACCUMULATE; // FP32
                    default: if (mul_counter >= 8) next_state = PE_ACCUMULATE;
                endcase
            end
            
            PE_ACCUMULATE: begin
                if (add_c_to_result)
                    next_state = PE_ADD_C;
                else if (k_step_count >= k_dim_i - 1)
                    next_state = PE_DONE;
                else
                    next_state = PE_IDLE;
            end
            
            PE_ADD_C: begin
                next_state = PE_DONE;
            end
            
            PE_DONE: begin
                pe_done_o = 1'b1;
                next_state = PE_IDLE;
            end
        endcase
    end
    
    
    
    // 数据处理逻辑
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            // 复位所有寄存器
            a_operand <= '0;
            b_operand <= '0;
            c_value <= '0;
            accum_int8 <= '0;
            accum_int16 <= '0;
            accum_fp16 <= '0;
            accum_fp32 <= '0;
            mul_result <= '0;
            partial_product <= '0;
            mul_sign <= 1'b0;
            exp_result <= 8'b0;
            result_out <= '0;
            k_step_count <= '0;
            max_mul_iterations <= '0;
            leading_zeros <= '0;
			
			fp_special_case <= 0;
            special_result <= '0;
        end else begin
            // 处理加载C数据
            if (load_c) begin
                c_value <= c_data_i;
            end
            
            case (current_state)
                PE_IDLE: begin
                    if (start) begin
                        // 加载操作数
                        a_operand <= a_data_i;
                        b_operand <= b_data_i;
                        
                        // 设置最大迭代次数
                        case (precision_mode)
                            3'b000: max_mul_iterations <= 4;   // INT4
                            3'b001: max_mul_iterations <= 8;   // INT8
                            3'b010, 3'b011: max_mul_iterations <= 11; // FP16
                            3'b100: max_mul_iterations <= 24;  // FP32
                            default: max_mul_iterations <= 8;
                        endcase
                        
                        // 重置累加器 (仅在第一个K步骤)
                        if (first_step) begin
                            k_step_count <= '0;
                            accum_int8 <= '0;
                            accum_int16 <= '0;
                            accum_fp16 <= '0;
                            accum_fp32 <= '0;
                        end
                        
                        // 初始化乘法
                        partial_product <= '0;
						fp_special_case <= 0; // 重置特殊情况标志
                        
                        // 浮点乘法初始化
                        if (precision_mode == 3'b010 || precision_mode == 3'b011) begin
                            // FP16乘法准备
                            mul_sign <= a_sign ^ b_sign;
                            exp_result <= a_exp + b_exp - 15; // FP16偏置是15
							   // FP16特殊情况检测
                            if ((a_exp == 5'h1F && a_mant != 0) || (b_exp == 5'h1F && b_mant != 0)) begin
                                // NaN处理 - 任何操作数是NaN，结果就是NaN
                                fp_special_case <= 1;
                                special_result[15:0] <= {1'b0, 5'h1F, 10'h200}; // 标准NaN
                                if (precision_mode == 3'b011)
                                    special_result[31:0] <= {1'b0, 8'hFF, 23'h400000}; // FP32 NaN
                            end else if (a_exp == 5'h1F || b_exp == 5'h1F) begin
                                // 无穷大处理
                                if ((a_exp == 0 && a_mant == 0) || (b_exp == 0 && b_mant == 0)) begin
                                    // 无穷大 × 零 = NaN
                                    fp_special_case <= 1;
                                    special_result[15:0] <= {1'b0, 5'h1F, 10'h200};
                                    if (precision_mode == 3'b011)
                                        special_result[31:0] <= {1'b0, 8'hFF, 23'h400000};
                                end else begin
                                    // 无穷大 × 非零数 = 符号相符的无穷大
                                    fp_special_case <= 1;
                                    special_result[15:0] <= {mul_sign, 5'h1F, 10'h0};
                                    if (precision_mode == 3'b011)
                                        special_result[31:0] <= {mul_sign, 8'hFF, 23'h0};
                                end
                            end else if ((a_exp == 0 && a_mant == 0) || (b_exp == 0 && b_mant == 0)) begin
                                // 零 × 任何数 = 零
                                fp_special_case <= 1;
                                special_result[15:0] <= {mul_sign, 15'h0};
                                if (precision_mode == 3'b011)
                                    special_result[31:0] <= {mul_sign, 31'h0};
                            end
                        end else if (precision_mode == 3'b100) begin
                            // FP32乘法准备
                            mul_sign <= a_sign ^ b_sign;
                            exp_result <= a_exp + b_exp - 127; // FP32偏置是127
							 // FP32特殊情况检测
                            if ((a_exp == 8'hFF && a_mant != 0) || (b_exp == 8'hFF && b_mant != 0)) begin
                                // NaN处理
                                fp_special_case <= 1;
                                special_result[31:0] <= {1'b0, 8'hFF, 23'h400000}; // 标准NaN
                            end else if (a_exp == 8'hFF || b_exp == 8'hFF) begin
                                // 无穷大处理
                                if ((a_exp == 0 && a_mant == 0) || (b_exp == 0 && b_mant == 0)) begin
                                    // 无穷大 × 零 = NaN
                                    fp_special_case <= 1;
                                    special_result[31:0] <= {1'b0, 8'hFF, 23'h400000};
                                end else begin
                                    // 无穷大 × 非零数 = 符号相符的无穷大
                                    fp_special_case <= 1;
                                    special_result[31:0] <= {mul_sign, 8'hFF, 23'h0};
                                end
                            end else if ((a_exp == 0 && a_mant == 0) || (b_exp == 0 && b_mant == 0)) begin
                                // 零 × 任何数 = 零
                                fp_special_case <= 1;
                                special_result[31:0] <= {mul_sign, 31'h0};
                            end
                        end
                    end
                end
                
                PE_MULTIPLY: begin
                    // 执行乘法迭代
                    case (precision_mode)
                        3'b000: begin
                            // INT4乘法 - 逐位乘加
                            if (mul_counter < 4) begin
                                if (b_int4[mul_counter])
                                    partial_product[7:0] <= partial_product[7:0] + (a_int4 << mul_counter);
                            end
                            // 负数处理 (补码乘法)
                            if (mul_counter == 3 && b_int4[3]) begin
                                partial_product[7:0] <= partial_product[7:0] - (a_int4 << 4);
                            end
                            
                            // 完成乘法
                            if (mul_counter >= 4 - 1) begin
							   if (a_int4 == -8 && b_int4 == -1)
                                    mul_result[7:0] <= 8'h7F; // 最大正数，因为-8 * -1 = 8 但INT4最大只能表示7
                            
                               else mul_result[7:0] <= partial_product[7:0]; 
                                mul_result[ACCUM_WIDTH-1:8] <= {(ACCUM_WIDTH-8){partial_product[7]}}; // 符号扩展
                            end
                        end
                        
                        3'b001: begin
                            // INT8乘法 - 逐位乘加
                            if (mul_counter < 8) begin
                                if (b_int8[mul_counter])
                                    partial_product[15:0] <= partial_product[15:0] + (a_int8 << mul_counter);
                            end
                            // 负数处理
                            if (mul_counter == 7 && b_int8[7]) begin
                                partial_product[15:0] <= partial_product[15:0] - (a_int8 << 8);
                            end
                            
                            // 完成乘法
                            if (mul_counter >= 8 - 1) begin							 
                                if (a_int8 == -128 && b_int8 == -1) 
                                    mul_result[15:0] <= 16'h7FFF; // 最大正数
                            
                                else mul_result[15:0] <= partial_product[15:0];
                                mul_result[ACCUM_WIDTH-1:16] <= {(ACCUM_WIDTH-16){partial_product[15]}}; // 符号扩展
                            end
                        end
                        
                        3'b010, 3'b011: begin
						 // FP16乘法 - 处理特殊情况
                            if (fp_special_case) begin
                                // 使用预先计算的特殊情况结果
                                if (precision_mode == 3'b010) begin
                                    mul_result[15:0] <= special_result[15:0];
                                    mul_result[ACCUM_WIDTH-1:16] <= 0;
                                end else begin
                                    mul_result[31:0] <= special_result[31:0];
                                end
                            end								
                            // FP16乘法 - 尾数乘法
                            else if (mul_counter < 11) begin
                                if (b_mant_full[mul_counter])
                                    partial_product[21:0] <= partial_product[21:0] + (a_mant_full[10:0] << mul_counter);
                            end
                            
                            // 规格化处理
                            if (mul_counter == 11 - 1) begin
                                // 检查溢出并规格化
                                if (partial_product[21]) begin
                                    // 需要右移
                                    partial_product[21:0] <= partial_product[21:0] >> 1;
                                    exp_result <= exp_result + 1;
                                end else begin
                                    // 计算前导零并左移规格化
                                    leading_zeros <= 0;
                                    for (int i = 20; i >= 0; i--) begin
                                        if (partial_product[i]) break;
                                        leading_zeros <= leading_zeros + 1;
                                    end
                                    
                                    if (leading_zeros > 0) begin
                                        partial_product[21:0] <= partial_product[21:0] << leading_zeros;
                                        exp_result <= exp_result - leading_zeros;
                                    end
                                end
                            end
                            
                            // 组装FP16结果
                            if (mul_counter >= 11) begin
                                if (precision_mode == 3'b010) begin
                                    // 普通FP16结果
                                    if (partial_product[21:0] == 0) begin
                                        mul_result[15:0] <= 16'b0; // 结果为零
                                    end else if (exp_result < 1) begin
                                        mul_result[15:0] <= {mul_sign, 15'b0}; // 下溢
                                    end else if (exp_result > 30) begin
                                        mul_result[15:0] <= {mul_sign, 5'b11111, 10'b0}; // 上溢
                                    end else begin
                                        mul_result[15:0] <= {mul_sign, exp_result[4:0], partial_product[19:10]};
                                    end
                                    mul_result[ACCUM_WIDTH-1:16] <= 0; // 高位清零
                                end else begin
                                    // FP16->FP32结果 (混合精度)
                                    logic [15:0] fp16_result;
                                    
                                    // 先构建FP16结果
                                    if (partial_product[21:0] == 0) begin
                                        fp16_result = 16'b0; // 结果为零
                                    end else if (exp_result < 1) begin
                                        fp16_result = {mul_sign, 15'b0}; // 下溢
                                    end else if (exp_result > 30) begin
                                        fp16_result = {mul_sign, 5'b11111, 10'b0}; // 上溢
                                    end else begin
                                        fp16_result = {mul_sign, exp_result[4:0], partial_product[19:10]};
                                    end
                                    
                                    // 然后扩展到FP32
                                    if (fp16_result[14:10] == 5'b11111) begin // Inf或NaN
                                        mul_result[31:0] <= {fp16_result[15], 8'hFF, fp16_result[9:0], 13'b0};
                                    end else if (fp16_result[14:10] == 5'b00000) begin // 零或非规格化
                                        if (fp16_result[9:0] == 10'b0) begin
                                            mul_result[31:0] <= {fp16_result[15], 31'b0}; // 零
                                        end else begin
                                            // 非规格化处理
                                            mul_result[31:0] <= {fp16_result[15], 8'd127 - 8'd14, fp16_result[9:0], 13'b0};
                                        end
                                    end else begin
                                        // 规格化数
                                        mul_result[31:0] <= {fp16_result[15], 8'd127 - 8'd15 + {3'b0, fp16_result[14:10]}, fp16_result[9:0], 13'b0};
                                    end
                                end
                            end
                        end
                        
                        3'b100:begin
                            // FP32尾数乘法 - 分批计算以避免过长循环
						    // FP32乘法 - 处理特殊情况
                            if (fp_special_case) begin
                                // 使用预先计算的特殊情况结果
                                mul_result[31:0] <= special_result[31:0];
								end else if (mul_counter < 24) begin
                                if (b_mant_full[mul_counter])
                                    partial_product <= partial_product + (a_mant_full << mul_counter);
                            end                           
                            // 规格化处理
                                if (mul_counter == 24 - 1) begin
                                    if (partial_product[47]) begin
                                    // 需要右移
                                    partial_product <= partial_product >> 1;
                                    exp_result <= exp_result + 1;
                                end else begin
                                    // 计算前导零并左移规格化
                                    leading_zeros <= 0;
                                    for (int i = 46; i >= 0; i--) begin
                                        if (partial_product[i]) break;
                                        leading_zeros <= leading_zeros + 1;
                                    end
                                    
                                    if (leading_zeros > 0) begin
                                        partial_product <= partial_product << leading_zeros;
                                        exp_result <= exp_result - leading_zeros;
                                    end
                                end
                            end
                            
                            // 组装FP32结果
                            if (mul_counter >= 24) begin
                                if (partial_product == 0) begin
                                    mul_result[31:0] <= {mul_sign, 31'b0}; // 结果为零
                                end else if (exp_result < 1) begin
                                    mul_result[31:0] <= {mul_sign, 31'b0}; // 下溢
                                end else if (exp_result > 254) begin
                                    mul_result[31:0] <= {mul_sign, 8'b11111111, 23'b0}; // 上溢
                                end else begin
                                    mul_result[31:0] <= {mul_sign, exp_result[7:0], partial_product[45:23]};
                                end
                            end
                    end
				endcase	
            end        
                
                
                PE_ACCUMULATE: begin
                    // 执行累加功能
                    if (first_step) begin
                        // 第一步初始化累加器
                        case (precision_mode)
                            3'b000: accum_int8 <= mul_result[7:0];
                            3'b001: accum_int16 <= mul_result[15:0];
                            3'b010: accum_fp16 <= mul_result[15:0];
                            3'b011: accum_fp32 <= mul_result[31:0];
                            3'b100: accum_fp32 <= mul_result[31:0];
                        endcase
                    end else begin
                        // 累加到现有值
                        case (precision_mode)
                            3'b000: accum_int8 <= accum_int8 + mul_result[7:0];
                            3'b001: accum_int16 <= accum_int16 + mul_result[15:0];
                            3'b010: accum_fp16 <= fp16_add(accum_fp16, mul_result[15:0]);
                            3'b011: accum_fp32 <= fp32_add(accum_fp32, mul_result[31:0]);
                            3'b100: accum_fp32 <= fp32_add(accum_fp32, mul_result[31:0]);
                        endcase
                    end
                    
                    // 更新K步计数
                    k_step_count <= k_step_count + 1;
                end
                
                PE_ADD_C: begin
                    // 将C加到最终结果
                    case (precision_mode)
                        3'b000: begin // INT8加法
                            logic signed [7:0] res8, c8, sum8;
                            res8 = accum_int8;
                            c8 = c_value[7:0];
                            sum8 = res8 + c8;
                            accum_int8 <= sum8;
                        end
                        
                        3'b001: begin // INT16加法
                            logic signed [15:0] res16, c16, sum16;
                            res16 = accum_int16;
                            c16 = c_value[15:0];
                            sum16 = res16 + c16;
                            accum_int16 <= sum16;
                        end
                        
                        3'b010: begin // FP16加法
                            accum_fp16 <= fp16_add(accum_fp16, c_value[15:0]);
                        end
                        
                        3'b011, 3'b100: begin // FP32加法
                            accum_fp32 <= fp32_add(accum_fp32, c_value);
                        end
                    endcase
                end
                
                PE_DONE: begin
                    // 输出最终结果
                    case (precision_mode)
                        3'b000: result_out <= {{(ACCUM_WIDTH-8){accum_int8[7]}}, accum_int8};
                        3'b001: result_out <= {{(ACCUM_WIDTH-16){accum_int16[15]}}, accum_int16};
                        3'b010: result_out <= {{(ACCUM_WIDTH-16){1'b0}}, accum_fp16};
                        3'b011, 3'b100: result_out <= accum_fp32;
                    endcase
                end
            endcase
        end
    end
	// FP16加法函数
    function logic [15:0] fp16_add(input logic [15:0] a, input logic [15:0] b);
        logic sign_a, sign_b, sign_res;        // 符号位
        logic [4:0] exp_a, exp_b, exp_res;     // 指数
        logic [9:0] mant_a, mant_b;            // 尾数
        logic [11:0] mant_a_full, mant_b_full; // 带隐藏位和保护位的尾数
        logic [11:0] mant_res;                 // 结果尾数
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
        mant_a_full = (exp_a == 0) ? {1'b0, mant_a, 1'b0} : {1'b1, mant_a, 1'b0};
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
    
    // FP32加法函数
    function logic [31:0] fp32_add(input logic [31:0] a, input logic [31:0] b);
        logic sign_a, sign_b, sign_res;        // 符号位
        logic [7:0] exp_a, exp_b, exp_res;     // 指数
        logic [22:0] mant_a, mant_b;           // 尾数
        logic [24:0] mant_a_full, mant_b_full; // 带隐藏位和保护位的尾数
        logic [24:0] mant_res;                 // 结果尾数
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
        mant_a_full = (exp_a == 0) ? {1'b0, mant_a, 1'b0} : {1'b1, mant_a, 1'b0};
        mant_b_full = (exp_b == 0) ? {1'b0, mant_b, 1'b0} : {1'b1, mant_b, 1'b0};
        
        // 对齐指数
        if (exp_a > exp_b) begin
            exp_diff = exp_a - exp_b;
            exp_res = exp_a;
            if (exp_diff > 25) exp_diff = 25; // 限制移位量
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
        if (mant_res == 0)
            return {sign_res, 31'b0};
            
        result = {sign_res, exp_res[7:0], mant_res_rounded};
        return result;
    endfunction
endmodule