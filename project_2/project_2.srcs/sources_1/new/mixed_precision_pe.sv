module mixed_precision_pe #(
    parameter integer PE_DATA_WIDTH = 32,        // PE 数据宽度 (当前为 32)
    parameter integer ACCUM_WIDTH = 32,          // 累加器宽度 (可能是 8/16/32 取决于模式)
    parameter integer PE_ID = 0,                 // PE 编号
    parameter integer K_BITS = 10                // K 维度宽度 (用于循环计数)
) (
    input clk,                                   // 时钟信号
    input reset,                                 // 复位信号

    // 来自 Core 的控制信号
    input start,                                 // 开始计算一个输出元素 (电平触发)
    input first_step,                            // 指示第一个步骤 (k=0)，用于重置累加器
    input [K_BITS-1:0] k_dim_i,                  // 累加步骤数 (K 维度)

    // 来自 Core 的数据输入
    input [PE_DATA_WIDTH-1:0] a_data_i,          // A 矩阵数据
    input [PE_DATA_WIDTH-1:0] b_data_i,          // B 矩阵数据
    input [ACCUM_WIDTH-1:0] c_data_i,            // C 矩阵数据
    
    // 精度模式: 
    // 000: INT4乘法+INT8累加
    // 001: INT8乘法+INT16累加
    // 010: FP16乘法+FP16累加
    // 011: FP16乘法+FP32累加 (混合精度)
    // 100: FP32乘法+FP32累加
    input [2:0] precision_mode,

    // 输出到 Core 的结果
    output logic [ACCUM_WIDTH-1:0] result_out,   // 最终计算结果 (不同精度，由Core解释)
    output logic pe_done_o                       // 计算完成信号, 完成时拉高一个周期
);

    // 添加综合指示，禁用DSP资源
    (* use_dsp = "no" *)

    // --- 内部信号 ---
    
    // 操作数寄存器
    logic [PE_DATA_WIDTH-1:0] a_operand, b_operand;     // 输入操作数
    logic signed [7:0] accum_int8;                      // INT8 累加器
    logic signed [15:0] accum_int16;                    // INT16 累加器
    logic [15:0] accum_fp16;                            // FP16 累加器 
    logic [31:0] accum_fp32;                            // FP32 累加器
    logic [ACCUM_WIDTH-1:0] mul_result;                 // 乘法结果
    
    // 状态机
    typedef enum logic [3:0] {
        PE_IDLE,                                 // 空闲状态
        PE_LOAD_OPERANDS,                        // 加载操作数
        PE_EXTRACT_OPERANDS,                     // 提取操作数
        PE_MUL_STEP1,                            // 乘法第一步
        PE_MUL_STEP2,                            // 乘法第二步
        PE_MUL_STEP3,                            // 乘法第三步
        PE_MUL_COMPLETE,                         // 乘法完成
        PE_ACCUMULATE,                           // 进行累加
        PE_NORMALIZE,                            // 规格化结果
        PE_DONE                                  // 完成
    } pe_state_t;
    
    pe_state_t current_state, next_state;
    logic [K_BITS-1:0] k_step_count;             // K 步计数器
    logic start_reg;                             // 寄存启动信号
    
    // 乘法器共享资源
    logic mul_sign;                               // 乘法结果符号位
    logic [7:0] exp_result;                       // 指数结果 (FP16/FP32)
    logic [47:0] mant_mul;                        // 尾数乘法结果 (支持最大FP32宽度)
    
    // 提取的操作数 - 用于不同精度格式
    logic signed [3:0] a_int4, b_int4;            // INT4操作数
    logic signed [7:0] a_int8, b_int8;            // INT8操作数
    logic [15:0] a_fp16, b_fp16;                  // FP16操作数
    logic [31:0] a_fp32, b_fp32;                  // FP32操作数
    
    // 浮点格式分量
    logic a_sign, b_sign;                         // 符号位
    logic [7:0] a_exp, b_exp;                     // 指数 (最多8位支持FP32)
    logic [22:0] a_mant, b_mant;                  // 尾数 (最多23位支持FP32)
    logic [23:0] a_mant_full, b_mant_full;        // 带隐藏位的尾数
    
    // 迭代乘法控制
    logic [5:0] mul_counter;                      // 乘法迭代计数器
    logic [5:0] max_mul_iterations;               // 最大迭代次数 (基于精度)
    
    // 中间结果存储
    logic [47:0] partial_product;                 // 乘法部分积
    logic [3:0] leading_zeros;                    // 前导零计数
    
    // 提取不同精度的操作数部分
    always_comb begin
        // 整数操作数提取
        a_int4 = a_operand[3:0];
        b_int4 = b_operand[3:0];
        a_int8 = a_operand[7:0];
        b_int8 = b_operand[7:0];
        
        // 浮点操作数提取
        a_fp16 = a_operand[15:0];
        b_fp16 = b_operand[15:0];
        a_fp32 = a_operand;
        b_fp32 = b_operand;
        
        // FP16 分量提取
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
        // FP32 分量提取
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

    // 状态寄存器
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            current_state <= PE_IDLE;
            mul_counter <= 0;
        end else begin
            current_state <= next_state;
            
            // 乘法迭代计数器控制
            if (current_state == PE_LOAD_OPERANDS || current_state == PE_IDLE) begin
                mul_counter <= 0;
            end else if (current_state == PE_MUL_STEP2) begin
                mul_counter <= mul_counter + 1;
            end
        end
    end

    // 下一状态组合逻辑
    always_comb begin
        next_state = current_state;
        pe_done_o = 1'b0;
        
        case (current_state)
            PE_IDLE: begin
                if (start_reg)
                    next_state = PE_LOAD_OPERANDS;
            end
            
            PE_LOAD_OPERANDS: begin
                next_state = PE_EXTRACT_OPERANDS;
            end
            
            PE_EXTRACT_OPERANDS: begin
                next_state = PE_MUL_STEP1;
            end
            
            PE_MUL_STEP1: begin
                next_state = PE_MUL_STEP2;
            end
            
            PE_MUL_STEP2: begin
                // 整数乘法迭代次数
                if (precision_mode == 3'b000 && mul_counter >= 4) 
                    next_state = PE_MUL_COMPLETE;
                // INT8乘法迭代次数
                else if (precision_mode == 3'b001 && mul_counter >= 8) 
                    next_state = PE_MUL_COMPLETE;
                // FP16乘法迭代次数
                else if ((precision_mode == 3'b010 || precision_mode == 3'b011) && mul_counter >= 11)
                    next_state = PE_MUL_STEP3;
                // FP32乘法迭代次数
                else if (precision_mode == 3'b100 && mul_counter >= 24)
                    next_state = PE_MUL_STEP3;
            end
            
            PE_MUL_STEP3: begin
                next_state = PE_MUL_COMPLETE;
            end
            
            PE_MUL_COMPLETE: begin
                next_state = PE_ACCUMULATE;
            end
            
            PE_ACCUMULATE: begin
                next_state = PE_NORMALIZE;
            end
            
            PE_NORMALIZE: begin
                next_state = PE_DONE;
            end
            
            PE_DONE: begin
                pe_done_o = 1'b1;
                if (k_dim_i > 0 && k_step_count == k_dim_i - 1) begin
                    next_state = PE_IDLE;
                end else if (k_dim_i == 0) begin
                    next_state = PE_IDLE;
                end else begin
                    next_state = PE_LOAD_OPERANDS;
                end
            end
            
            default: next_state = PE_IDLE;
        endcase
    end

    // 数据寄存器和乘法器逻辑
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            // 复位所有寄存器
            a_operand <= '0;
            b_operand <= '0;
            accum_int8 <= '0;
            accum_int16 <= '0;
            accum_fp16 <= '0;
            accum_fp32 <= '0;
            mul_result <= '0;
            partial_product <= '0;
            mul_sign <= 1'b0;
            exp_result <= 8'b0;
            mant_mul <= '0;
            result_out <= '0;
            k_step_count <= '0;
            start_reg <= 1'b0;
            max_mul_iterations <= '0;
            leading_zeros <= '0;
        end else begin
            start_reg <= start;
            
            case (current_state)
                PE_IDLE: begin
                    if (start && !start_reg) begin
                        // 初始化
                        k_step_count <= 0;
                    end
                end
                
                PE_LOAD_OPERANDS: begin
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
                end
                
                PE_EXTRACT_OPERANDS: begin
                    // 浮点运算前处理
                    if (precision_mode == 3'b010 || precision_mode == 3'b011) begin
                        // FP16 指数预加
                        mul_sign <= a_sign ^ b_sign;
                        exp_result <= a_exp + b_exp - 15; // FP16偏置是15
                    end else if (precision_mode == 3'b100) begin
                        // FP32 指数预加
                        mul_sign <= a_sign ^ b_sign;
                        exp_result <= a_exp + b_exp - 127; // FP32偏置是127
                    end
                    
                    // 初始化乘法
                    partial_product <= '0;
                end
                
                PE_MUL_STEP1: begin
                    // 初始化乘法器
                    if (precision_mode == 3'b000) begin
                        // INT4 乘法准备
                    end else if (precision_mode == 3'b001) begin
                        // INT8 乘法准备
                    end else if (precision_mode == 3'b010 || precision_mode == 3'b011 || precision_mode == 3'b100) begin
                        // 浮点乘法尾数准备
                    end
                end
                
                PE_MUL_STEP2: begin
                    // 执行乘法迭代
                    case (precision_mode)
                        3'b000: begin
                            // INT4 乘法使用移位和加法
                            if (mul_counter < 4) begin
                                if (b_int4[mul_counter])
                                    partial_product[7:0] <= partial_product[7:0] + (a_int4 << mul_counter);
                            end
                            // 负数处理
                            if (mul_counter == 3 && b_int4[3]) begin
                                partial_product[7:0] <= partial_product[7:0] - (a_int4 << 4);
                            end
                        end
                        
                        3'b001: begin
                            // INT8 乘法使用移位和加法
                            if (mul_counter < 8) begin
                                if (b_int8[mul_counter])
                                    partial_product[15:0] <= partial_product[15:0] + (a_int8 << mul_counter);
                            end
                            // 负数处理
                            if (mul_counter == 7 && b_int8[7]) begin
                                partial_product[15:0] <= partial_product[15:0] - (a_int8 << 8);
                            end
                        end
                        
                        3'b010, 3'b011: begin
                            // FP16尾数乘法
                            if (mul_counter < 11) begin
                               if ( b_mant_full[mul_counter])// if (mul_counter < b_mant_full[10:0]'size && b_mant_full[mul_counter])
                                    partial_product[21:0] <= partial_product[21:0] + (a_mant_full[10:0] << mul_counter);
                            end
                        end
                        
                        3'b100: begin
                            // FP32尾数乘法 - 分批计算以避免过长循环
                            if (mul_counter < 24) begin
                                if (b_mant_full[mul_counter])
                                    partial_product <= partial_product + (a_mant_full << mul_counter);
                            end
                        end
                    endcase
                end
                
                PE_MUL_STEP3: begin
                    // 浮点结果规格化 - 找前导零和调整指数
                    if (precision_mode == 3'b010 || precision_mode == 3'b011) begin
                        // FP16 规格化
                        // 计算前导零
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
                    end else if (precision_mode == 3'b100) begin
                        // FP32 规格化 - 类似逻辑
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
                end
                
                PE_MUL_COMPLETE: begin
                    // 完成乘法，创建结果
                    case (precision_mode)
                        3'b000: begin
                            // INT4 乘法结果
                            mul_result[7:0] <= partial_product[7:0];
                            mul_result[ACCUM_WIDTH-1:8] <= {(ACCUM_WIDTH-8){partial_product[7]}};  // 符号扩展
                        end
                        3'b001: begin
                            // INT8 乘法结果
                            mul_result[15:0] <= partial_product[15:0];
                            mul_result[ACCUM_WIDTH-1:16] <= {(ACCUM_WIDTH-16){partial_product[15]}}; // 符号扩展
                        end
                        3'b010: begin
                            // FP16 乘法结果
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
                        end
                        3'b011: begin
                            // FP16 -> FP32 扩展精度
                            // 先构建FP16结果
                            logic [15:0] fp16_result;
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
                        3'b100: begin
                            // FP32 乘法结果
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
                         3'b010: begin
                             // 完整的FP16加法
                             logic sign_a, sign_b, sign_res;
                             logic [4:0] exp_a, exp_b, exp_res;
                             logic [9:0] mant_a, mant_b;
                             logic [11:0] mant_a_full, mant_b_full;
                             logic [11:0] mant_res;
                             int exp_diff;
                             
                             // 提取组件
                             sign_a = accum_fp16[15];
                             exp_a = accum_fp16[14:10];
                             mant_a = accum_fp16[9:0];
                             sign_b = mul_result[15];
                             exp_b = mul_result[14:10];
                             mant_b = mul_result[9:0];
                             
                             // 特殊情况处理
                             // 如果有一个操作数是零
                             if (exp_a == 5'b0 && mant_a == 10'b0) begin
                                 accum_fp16 <= mul_result[15:0];
                             end else if (exp_b == 5'b0 && mant_b == 10'b0) begin
                                 accum_fp16 <= accum_fp16;
                             end 
                             // 无穷大和NaN处理
                             else if (exp_a == 5'h1F) begin
                                 if (mant_a != 0) begin // NaN传播
                                     accum_fp16 <= accum_fp16;
                                 end else if (exp_b == 5'h1F && mant_b == 0 && sign_a != sign_b) begin
                                     accum_fp16 <= 16'h7E00; // +Inf + (-Inf) = NaN
                                 end else begin
                                     accum_fp16 <= accum_fp16; // Inf + 任何数 = Inf
                                 end
                             end
                             else if (exp_b == 5'h1F) begin 
                                 if (mant_b != 0) begin // NaN传播
                                     accum_fp16 <= mul_result[15:0];
                                 end else begin
                                     accum_fp16 <= mul_result[15:0]; // Inf + 任何数 = Inf
                                 end
                             end
                             else begin
                                 // 正常情况下的加法
                                 
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
                                     logic [3:0] leading_zeros = 0;
                                     
                                     if (mant_res[9:0] == 0) leading_zeros = 10;
                                     else if (mant_res[9:1] == 0) leading_zeros = 9;
                                     else if (mant_res[9:2] == 0) leading_zeros = 8;
                                     else if (mant_res[9:3] == 0) leading_zeros = 7;
                                     else if (mant_res[9:4] == 0) leading_zeros = 6;
                                     else if (mant_res[9:5] == 0) leading_zeros = 5;
                                     else if (mant_res[9:6] == 0) leading_zeros = 4;
                                     else if (mant_res[9:7] == 0) leading_zeros = 3;
                                     else if (mant_res[9:8] == 0) leading_zeros = 2;
                                     else if (mant_res[9] == 0) leading_zeros = 1;
                                     
                                     if (leading_zeros == 10) begin
                                         // 结果为零
                                         accum_fp16 <= {sign_res, 15'b0};
                                     end else if (leading_zeros > 0 && leading_zeros <= exp_res) begin
                                         mant_res = mant_res << leading_zeros;
                                         exp_res = exp_res - leading_zeros;
                                         
                                         // 组装结果
                                         accum_fp16 <= {sign_res, exp_res[4:0], mant_res[9:0]};
                                     end else if (leading_zeros > exp_res) begin
                                         // 下溢
                                         mant_res = mant_res << exp_res;
                                         exp_res = 0;
                                         
                                         // 组装非规格化结果
                                         accum_fp16 <= {sign_res, 5'b0, mant_res[9:0]};
                                     end else begin
                                         // 正常情况
                                         accum_fp16 <= {sign_res, exp_res[4:0], mant_res[9:0]};
                                     end
                                 end else begin
                                     // 溢出检查
                                     if (exp_res >= 31) begin
                                         accum_fp16 <= {sign_res, 5'h1F, 10'b0}; // 溢出为无穷大
                                     end else begin
                                         // 组装正常结果
                                         accum_fp16 <= {sign_res, exp_res[4:0], mant_res[9:0]};
                                     end
                                 end
                             end
                         end
                         
                         3'b011, 3'b100: begin
                             // 完整的FP32加法
                             logic sign_a, sign_b, sign_res;
                             logic [7:0] exp_a, exp_b, exp_res;
                             logic [22:0] mant_a, mant_b;
                             logic [24:0] mant_a_full, mant_b_full;
                             logic [24:0] mant_res;
                             int exp_diff;
                             
                             // 提取组件
                             sign_a = accum_fp32[31];
                             exp_a = accum_fp32[30:23];
                             mant_a = accum_fp32[22:0];
                             sign_b = mul_result[31];
                             exp_b = mul_result[30:23];
                             mant_b = mul_result[22:0];
                             
                             // 特殊情况处理
                             // 如果有一个操作数是零
                             if (exp_a == 8'b0 && mant_a == 23'b0) begin
                                 accum_fp32 <= mul_result[31:0];
                             end else if (exp_b == 8'b0 && mant_b == 23'b0) begin
                                 accum_fp32 <= accum_fp32;
                             end 
                             // 无穷大和NaN处理
                             else if (exp_a == 8'hFF) begin
                                 if (mant_a != 0) begin // NaN传播
                                     accum_fp32 <= accum_fp32;
                                 end else if (exp_b == 8'hFF && mant_b == 0 && sign_a != sign_b) begin
                                     accum_fp32 <= 32'h7FC00000; // +Inf + (-Inf) = NaN
                                 end else begin
                                     accum_fp32 <= accum_fp32; // Inf + 任何数 = Inf
                                 end
                             end
                             else if (exp_b == 8'hFF) begin 
                                 if (mant_b != 0) begin // NaN传播
                                     accum_fp32 <= mul_result[31:0];
                                 end else begin
                                     accum_fp32 <= mul_result[31:0]; // Inf + 任何数 = Inf
                                 end
                             end
                             else begin
                                 // 正常情况下的加法
                                 
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
                                 if (mant_res[24]) begin // 需要右移一位
                                     mant_res = mant_res >> 1;
                                     exp_res = exp_res + 1;
                                 end else if (mant_res[23] == 0) begin
                                     // 处理前导零
                                     logic [4:0] leading_zeros = 0;
                                     
                                     if (mant_res[22:0] == 0) leading_zeros = 23;
                                     else begin
                                         // 确定前导零的数量
                                         // 分组确定以加快计算
                                         if (mant_res[22:16] == 0) begin
                                             leading_zeros = leading_zeros + 7;
                                             if (mant_res[15:8] == 0) begin
                                                 leading_zeros = leading_zeros + 8;
                                                 if (mant_res[7:4] == 0) begin
                                                     leading_zeros = leading_zeros + 4;
                                                     if (mant_res[3:2] == 0) begin
                                                         leading_zeros = leading_zeros + 2;
                                                         if (mant_res[1] == 0)
                                                             leading_zeros = leading_zeros + 1;
                                                     end else if (mant_res[3] == 0) begin
                                                         leading_zeros = leading_zeros + 1;
                                                     end
                                                 end else if (mant_res[7:6] == 0) begin
                                                     leading_zeros = leading_zeros + 2;
                                                     if (mant_res[5] == 0)
                                                         leading_zeros = leading_zeros + 1;
                                                 end else if (mant_res[7] == 0) begin
                                                     leading_zeros = leading_zeros + 1;
                                                 end
                                             end else if (mant_res[15:12] == 0) begin
                                                 leading_zeros = leading_zeros + 4;
                                                 if (mant_res[11:10] == 0) begin
                                                     leading_zeros = leading_zeros + 2;
                                                     if (mant_res[9] == 0)
                                                         leading_zeros = leading_zeros + 1;
                                                 end else if (mant_res[11] == 0) begin
                                                     leading_zeros = leading_zeros + 1;
                                                 end
                                             end else if (mant_res[15:14] == 0) begin
                                                 leading_zeros = leading_zeros + 2;
                                                 if (mant_res[13] == 0)
                                                     leading_zeros = leading_zeros + 1;
                                             end else if (mant_res[15] == 0) begin
                                                 leading_zeros = leading_zeros + 1;
                                             end
                                         end else if (mant_res[22:20] == 0) begin
                                             leading_zeros = leading_zeros + 3;
                                             if (mant_res[19:18] == 0) begin
                                                 leading_zeros = leading_zeros + 2;
                                                 if (mant_res[17] == 0)
                                                     leading_zeros = leading_zeros + 1;
                                             end else if (mant_res[19] == 0) begin
                                                 leading_zeros = leading_zeros + 1;
                                             end
                                         end else if (mant_res[22:21] == 0) begin
                                             leading_zeros = leading_zeros + 2;
                                             if (mant_res[20] == 0)
                                                 leading_zeros = leading_zeros + 1;
                                         end else if (mant_res[22] == 0) begin
                                             leading_zeros = leading_zeros + 1;
                                         end
                                     end
                                     
                                     if (leading_zeros == 23) begin
                                         // 结果为零
                                         accum_fp32 <= {sign_res, 31'b0};
                                     end else if (leading_zeros > 0 && leading_zeros <= exp_res) begin
                                         mant_res = mant_res << leading_zeros;
                                         exp_res = exp_res - leading_zeros;
                                         
                                         // 组装结果
                                         accum_fp32 <= {sign_res, exp_res[7:0], mant_res[22:0]};
                                     end else if (leading_zeros > exp_res) begin
                                         // 下溢
                                         mant_res = mant_res << exp_res;
                                         exp_res = 0;
                                         
                                         // 组装非规格化结果
                                         accum_fp32 <= {sign_res, 8'b0, mant_res[22:0]};
                                     end else begin
                                         // 正常情况
                                         accum_fp32 <= {sign_res, exp_res[7:0], mant_res[22:0]};
                                     end
                                 end else begin
                                     // 溢出检查
                                     if (exp_res >= 255) begin
                                         accum_fp32 <= {sign_res, 8'hFF, 23'b0}; // 溢出为无穷大
                                     end else begin
                                         // 组装正常结果
                                         accum_fp32 <= {sign_res, exp_res[7:0], mant_res[22:0]};
                                     end
                                 end
                             end
                         end
                     endcase
                 end
                end
                
                PE_NORMALIZE: begin
                    // 处理浮点结果的规格化 - 这里简化处理
                    // 实际实现应该包含完整的规格化逻辑
                end
                
                PE_DONE: begin
                    // 计算完成，输出最终结果
                    if (k_dim_i == 0) begin
                        result_out <= '0;
                    end else begin
                        case (precision_mode)
                            3'b000: result_out <= {{(ACCUM_WIDTH-8){accum_int8[7]}}, accum_int8};
                            3'b001: result_out <= {{(ACCUM_WIDTH-16){accum_int16[15]}}, accum_int16};
                            3'b010: result_out <= {{(ACCUM_WIDTH-16){1'b0}}, accum_fp16};
                            3'b011: result_out <= accum_fp32;
                            3'b100: result_out <= accum_fp32;
                            default: result_out <= {{(ACCUM_WIDTH-8){accum_int8[7]}}, accum_int8};
                        endcase
                    end
                end
            endcase
        end
    end

endmodule