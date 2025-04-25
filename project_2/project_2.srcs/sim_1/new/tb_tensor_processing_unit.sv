`timescale 1ns / 1ps

module tb_tensor_processing_unit;
    // 参数设置
    parameter AXI_DATA_WIDTH    = 64;
    parameter AXI_ADDR_WIDTH    = 32;
    parameter AXI_ID_WIDTH      = 1;
    parameter AXI_USER_WIDTH    = 1;
    parameter PE_DATA_WIDTH     = 32;
    parameter ACCUM_WIDTH       = 32;
    parameter INPUT_BUF_DEPTH   = 1024;
    parameter OUTPUT_BUF_DEPTH  = 1024;
    parameter CORE_ADDR_WIDTH   = $clog2(INPUT_BUF_DEPTH);
    parameter PE_ARRAY_HEIGHT   = 16;
    parameter PE_ARRAY_WIDTH    = 16;
    parameter MATRIX_SIZE_WIDTH = 32;
    
    // 基本信号
    reg  aclk;
    reg  aresetn;
    
    // AXI4-Lite 控制接口
    reg                        s_axil_awvalid;
    wire                       s_axil_awready;
    reg  [AXI_ADDR_WIDTH-1:0]  s_axil_awaddr;
    reg                        s_axil_wvalid;
    wire                       s_axil_wready;
    reg  [31:0]                s_axil_wdata;
    reg  [3:0]                 s_axil_wstrb;
    wire                       s_axil_bvalid;
    reg                        s_axil_bready;
    wire [1:0]                 s_axil_bresp;
    reg                        s_axil_arvalid;
    wire                       s_axil_arready;
    reg  [AXI_ADDR_WIDTH-1:0]  s_axil_araddr;
    wire                       s_axil_rvalid;
    reg                        s_axil_rready;
    wire [31:0]                s_axil_rdata;
    wire [1:0]                 s_axil_rresp;
    
    // AXI4-Full (读取接口)
    wire [AXI_ID_WIDTH-1:0]    m_axi_read_arid;
    wire [AXI_ADDR_WIDTH-1:0]  m_axi_read_araddr;
    wire [7:0]                 m_axi_read_arlen;
    wire [2:0]                 m_axi_read_arsize;
    wire [1:0]                 m_axi_read_arburst;
    wire                       m_axi_read_arlock;
    wire [3:0]                 m_axi_read_arcache;
    wire [2:0]                 m_axi_read_arprot;
    wire [3:0]                 m_axi_read_arqos;
    wire                       m_axi_read_arvalid;
    reg                        m_axi_read_arready;
    reg  [AXI_ID_WIDTH-1:0]    m_axi_read_rid;
    reg  [AXI_DATA_WIDTH-1:0]  m_axi_read_rdata;
    reg  [1:0]                 m_axi_read_rresp;
    reg                        m_axi_read_rlast;
    reg                        m_axi_read_rvalid;
    wire                       m_axi_read_rready;
    
    // AXI4-Full (写入接口)
    wire [AXI_ID_WIDTH-1:0]    m_axi_awid;
    wire [AXI_ADDR_WIDTH-1:0]  m_axi_awaddr;
    wire [7:0]                 m_axi_awlen;
    wire [2:0]                 m_axi_awsize;
    wire [1:0]                 m_axi_awburst;
    wire                       m_axi_awlock;
    wire [3:0]                 m_axi_awcache;
    wire [2:0]                 m_axi_awprot;
    wire [3:0]                 m_axi_awqos;
    wire                       m_axi_awvalid;
    reg                        m_axi_awready;
    wire [AXI_DATA_WIDTH-1:0]  m_axi_wdata;
    wire [AXI_DATA_WIDTH/8-1:0]m_axi_wstrb;
    wire                       m_axi_wlast;
    wire                       m_axi_wvalid;
    reg                        m_axi_wready;
    reg  [AXI_ID_WIDTH-1:0]    m_axi_bid;
    reg  [1:0]                 m_axi_bresp;
    reg                        m_axi_bvalid;
    wire                       m_axi_bready;

    // 被测设备实例化
    tensor_processing_unit #(
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
        .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
        .AXI_ID_WIDTH(AXI_ID_WIDTH),
        .AXI_USER_WIDTH(AXI_USER_WIDTH),
        .PE_DATA_WIDTH(PE_DATA_WIDTH),
        .ACCUM_WIDTH(ACCUM_WIDTH),
        .INPUT_BUF_DEPTH(INPUT_BUF_DEPTH),
        .OUTPUT_BUF_DEPTH(OUTPUT_BUF_DEPTH),
        .CORE_ADDR_WIDTH(CORE_ADDR_WIDTH),
        .PE_ARRAY_HEIGHT(PE_ARRAY_HEIGHT),
        .PE_ARRAY_WIDTH(PE_ARRAY_WIDTH),
        .MATRIX_SIZE_WIDTH(MATRIX_SIZE_WIDTH)
    ) dut (.*);

    // 内存模拟 (A, B, C和D矩阵)
    reg [PE_DATA_WIDTH-1:0] mem_a[0:255];  // 16x16矩阵 = 256个元素
    reg [PE_DATA_WIDTH-1:0] mem_b[0:255];
    reg [ACCUM_WIDTH-1:0]   mem_c[0:255];
    reg [ACCUM_WIDTH-1:0]   mem_d[0:255];
    
    // 测试数据存储地址
    localparam A_BASE_ADDR = 32'h1000_0000;
    localparam B_BASE_ADDR = 32'h2000_0000;
    localparam C_BASE_ADDR = 32'h3000_0000;
    localparam D_BASE_ADDR = 32'h4000_0000;
    
    // AXI-Lite寄存器地址
    localparam ADDR_CONTROL      = 32'h00;
    localparam ADDR_MATRIX_SIZE  = 32'h04;
    localparam ADDR_OP_MODE      = 32'h08;
    localparam ADDR_INPUT_A_ADDR = 32'h10;
    localparam ADDR_INPUT_B_ADDR = 32'h14;
    localparam ADDR_INPUT_C_ADDR = 32'h18;
    localparam ADDR_OUTPUT_D_ADDR= 32'h1C;
    
    // 内部状态变量
    reg [31:0] current_read_addr;
    reg [7:0]  current_read_len;
    reg [7:0]  read_count;
    reg read_active;
    
    reg [31:0] current_write_addr;
    reg [7:0]  current_write_len;
    reg [7:0]  write_count;
    reg write_active;
    
    // 时钟生成
    initial begin
        aclk = 0;
        forever #5 aclk = ~aclk; // 100MHz时钟
    end
    
    // 函数1: AXI-Lite写寄存器
    task write_reg;
        input [31:0] addr;
        input [31:0] data;
        begin
            // 先等待一个时钟周期
            @(posedge aclk);
            
            // AW通道
            s_axil_awvalid = 1;
            s_axil_awaddr = addr;
            s_axil_wvalid = 1;
            s_axil_wdata = data;
            s_axil_wstrb = 4'b1111; // 全部写入
            
            // 等待握手完成
            while(!(s_axil_awready && s_axil_wready)) @(posedge aclk);
            @(posedge aclk);
            
            // 释放地址和数据通道
            s_axil_awvalid = 0;
            s_axil_wvalid = 0;
            
            // B通道接收响应
            s_axil_bready = 1;
            while(!s_axil_bvalid) @(posedge aclk);
            @(posedge aclk);
            s_axil_bready = 0;
            
            $display("already write reg addr=0x%h, data=0x%h", addr, data);
        end
    endtask
    
    // 函数2: AXI-Lite读寄存器
    task read_reg;
        input [31:0] addr;
        output [31:0] data;
        begin
            // 先等待一个时钟周期
            @(posedge aclk);
            
            // AR通道
            s_axil_arvalid = 1;
            s_axil_araddr = addr;
            
            // 等待握手完成
            while(!s_axil_arready) @(posedge aclk);
            @(posedge aclk);
            
            // 释放地址通道
            s_axil_arvalid = 0;
            
            // R通道接收数据
            s_axil_rready = 1;
            while(!s_axil_rvalid) @(posedge aclk);
            data = s_axil_rdata;
            @(posedge aclk);
            s_axil_rready = 0;
            
            $display("already read reg addr=0x%h, data=0x%h", addr, data);
        end
    endtask
    
    // 处理AXI4读请求 - 优化版
always @(posedge aclk) begin
    if (!aresetn) begin
        // 复位所有信号
        m_axi_read_arready <= 0;
        m_axi_read_rvalid <= 0;
        m_axi_read_rlast <= 0;
        m_axi_read_rid <= 0;
        m_axi_read_rresp <= 0;
        m_axi_read_rdata <= 0;
        read_active <= 0;
        read_count <= 0;
        current_read_addr <= 0;
        current_read_len <= 0;
    end else begin
        // 地址通道处理 - 优先处理地址握手
        if (m_axi_read_arvalid && !read_active) begin
            // 接收新的读请求
            m_axi_read_arready <= 1;
            if (m_axi_read_arready) begin
                // 地址握手完成，保存请求信息
                current_read_addr <= m_axi_read_araddr;
                current_read_len <= m_axi_read_arlen;
                read_count <= 0;
                read_active <= 1;
                m_axi_read_arready <= 0; // 立即清除arready防止接收新请求
            end
        end else begin
            // 无新请求或正在处理，清除arready
            m_axi_read_arready <= 0;
        end
        
        // 数据通道处理
        if (read_active) begin
            // 数据通道状态转换
            if (m_axi_read_rvalid && m_axi_read_rready) begin
                // 当前数据被接收，准备下一个或完成传输
                if (read_count == current_read_len) begin
                    // 完成整个burst传输
                    m_axi_read_rvalid <= 0;
                    m_axi_read_rlast <= 0;
                    read_active <= 0; // 释放通道，可以接收新的请求
                end else begin
                    // 继续下一个beat
                    read_count <= read_count + 1;
                    m_axi_read_rlast <= (read_count + 1 == current_read_len);
                    
                    // 准备下一个数据
                    if (current_read_addr >= A_BASE_ADDR && current_read_addr < B_BASE_ADDR) begin
                        // 读取A矩阵数据
                        for (int i = 0; i < AXI_DATA_WIDTH/PE_DATA_WIDTH; i++) begin
                            int index = ((current_read_addr - A_BASE_ADDR) >> 2) + read_count*(AXI_DATA_WIDTH/PE_DATA_WIDTH) + i ;
                            if (index < 256) begin
                                m_axi_read_rdata[i*PE_DATA_WIDTH +: PE_DATA_WIDTH] <= mem_a[index];
                            end else begin
                                m_axi_read_rdata[i*PE_DATA_WIDTH +: PE_DATA_WIDTH] <= 0;
                            end
                        end
                    end else if (current_read_addr >= B_BASE_ADDR && current_read_addr < C_BASE_ADDR) begin
                        // 读取B矩阵数据
                        for (int i = 0; i < AXI_DATA_WIDTH/PE_DATA_WIDTH; i++) begin
                            int index = ((current_read_addr - B_BASE_ADDR) >> 2) + read_count*(AXI_DATA_WIDTH/PE_DATA_WIDTH) + i ;
                            if (index < 256) begin
                                m_axi_read_rdata[i*PE_DATA_WIDTH +: PE_DATA_WIDTH] <= mem_b[index];
                            end else begin
                                m_axi_read_rdata[i*PE_DATA_WIDTH +: PE_DATA_WIDTH] <= 0;
                            end
                        end
                    end else if (current_read_addr >= C_BASE_ADDR && current_read_addr < D_BASE_ADDR) begin
                        // 读取C矩阵数据
                        for (int i = 0; i < AXI_DATA_WIDTH/ACCUM_WIDTH; i++) begin
                            int index = ((current_read_addr - C_BASE_ADDR) >> 2) + read_count*(AXI_DATA_WIDTH/ACCUM_WIDTH) + i ;
                            if (index < 256) begin
                                m_axi_read_rdata[i*ACCUM_WIDTH +: ACCUM_WIDTH] <= mem_c[index];
                            end else begin
                                m_axi_read_rdata[i*ACCUM_WIDTH +: ACCUM_WIDTH] <= 0;
                            end
                        end
                    end
                end
            end else if (!m_axi_read_rvalid) begin
                // 没有未处理的数据，准备第一个或新的数据
                m_axi_read_rvalid <= 1;
                m_axi_read_rid <= 0;
                m_axi_read_rresp <= 2'b00; // OKAY
                
                // 根据请求地址确定数据来源
                if (current_read_addr >= A_BASE_ADDR && current_read_addr < B_BASE_ADDR) begin
                    // 读取A矩阵数据
                    for (int i = 0; i < AXI_DATA_WIDTH/PE_DATA_WIDTH; i++) begin
                        int index = ((current_read_addr - A_BASE_ADDR) >> 2) + read_count*(AXI_DATA_WIDTH/PE_DATA_WIDTH) + i;
                        if (index < 256) begin
                            m_axi_read_rdata[i*PE_DATA_WIDTH +: PE_DATA_WIDTH] <= mem_a[index];
                        end else begin
                            m_axi_read_rdata[i*PE_DATA_WIDTH +: PE_DATA_WIDTH] <= 0;
                        end
                    end
                end else if (current_read_addr >= B_BASE_ADDR && current_read_addr < C_BASE_ADDR) begin
                    // 读取B矩阵数据
                    for (int i = 0; i < AXI_DATA_WIDTH/PE_DATA_WIDTH; i++) begin
                        int index = ((current_read_addr - B_BASE_ADDR) >> 2) + read_count*(AXI_DATA_WIDTH/PE_DATA_WIDTH) + i;
                        if (index < 256) begin
                            m_axi_read_rdata[i*PE_DATA_WIDTH +: PE_DATA_WIDTH] <= mem_b[index];
                        end else begin
                            m_axi_read_rdata[i*PE_DATA_WIDTH +: PE_DATA_WIDTH] <= 0;
                        end
                    end
                end else if (current_read_addr >= C_BASE_ADDR && current_read_addr < D_BASE_ADDR) begin
                    // 读取C矩阵数据
                    for (int i = 0; i < AXI_DATA_WIDTH/ACCUM_WIDTH; i++) begin
                        int index = ((current_read_addr - C_BASE_ADDR) >> 2) + read_count*(AXI_DATA_WIDTH/ACCUM_WIDTH) + i;
                        if (index < 256) begin
                            m_axi_read_rdata[i*ACCUM_WIDTH +: ACCUM_WIDTH] <= mem_c[index];
                        end else begin
                            m_axi_read_rdata[i*ACCUM_WIDTH +: ACCUM_WIDTH] <= 0;
                        end
                    end
                end
                
                // 设置最后一个beat标志
                m_axi_read_rlast <= (read_count == current_read_len);
            end
        end
    end
end

// 处理AXI4写请求 - 优化版
always @(posedge aclk) begin
    if (!aresetn) begin
        // 复位所有信号
        m_axi_awready <= 0;
        m_axi_wready <= 0;
        m_axi_bvalid <= 0;
        m_axi_bid <= 0;
        m_axi_bresp <= 0;
        write_active <= 0;
        write_count <= 0;
        current_write_addr <= 0;
        current_write_len <= 0;
    end else begin
        // 地址通道处理
        if (m_axi_awvalid && !write_active) begin
            // 接收新的写请求
            m_axi_awready <= 1;
            if (m_axi_awready) begin
                // 地址握手完成，保存请求信息
                current_write_addr <= m_axi_awaddr;
                current_write_len <= m_axi_awlen;
                write_count <= 0;
                write_active <= 1;
                m_axi_awready <= 0; // 立即清除awready
                m_axi_wready <= 1;  // 准备接收数据
            end
        end else begin
            m_axi_awready <= 0;
        end
        
        // 数据通道处理
        if (write_active) begin
            // 确保wready始终有效直到传输完成
            if (!m_axi_wready) m_axi_wready <= 1;
            
            // 处理数据写入
            if (m_axi_wvalid && m_axi_wready) begin
                // 将数据写入内存
                if (current_write_addr >= D_BASE_ADDR) begin
                    // 写入D矩阵
                    for (int i = 0; i < AXI_DATA_WIDTH/ACCUM_WIDTH; i++) begin
                        int index = ((current_write_addr - D_BASE_ADDR) >> 2) + write_count*(AXI_DATA_WIDTH/ACCUM_WIDTH) + i;
                        if (index < 256 && m_axi_wstrb[i*(ACCUM_WIDTH/8) +: (ACCUM_WIDTH/8)] == {(ACCUM_WIDTH/8){1'b1}}) begin
                            mem_d[index] <= m_axi_wdata[i*ACCUM_WIDTH +: ACCUM_WIDTH];
                            $display("D[%0d] = 0x%h", index, m_axi_wdata[i*ACCUM_WIDTH +: ACCUM_WIDTH]);
                        end
                    end
                end
                
                // 更新计数
                write_count <= write_count + 1;
                
                // 检查最后一个数据
                if (m_axi_wlast) begin
                    m_axi_wready <= 0;     // 不再接收数据
                    m_axi_bvalid <= 1;     // 发送写响应
                    m_axi_bid <= 0;
                    m_axi_bresp <= 2'b00;  // OKAY
                    // 不立即清除write_active，等待响应被接收
                end
            end
            
            // 响应通道处理
            if (m_axi_bvalid && m_axi_bready) begin
                m_axi_bvalid <= 0;     // 清除响应有效标志
                write_active <= 0;     // 释放通道，可以接收新请求
            end
        end
    end
end
    integer expected_d[0:255];
	integer errors = 0;
	integer actual;
	reg [31:0] status;
    // 主测试流程
    initial begin
        // 初始化信号
        aresetn = 0;
        s_axil_awvalid = 0;
        s_axil_wvalid = 0;
        s_axil_bready = 0;
        s_axil_arvalid = 0;
        s_axil_rready = 0;
        
        // 等待一段时间后释放复位
        #100;
        aresetn = 1;
        #50;
        
        // 初始化测试矩阵
        $display("Initial test MATRIX...");
        for (int i = 0; i < 16; i++) begin
            for (int j = 0; j < 16; j++) begin
                // A矩阵：所有行的值相同 (使用行索引作为INT4值)
                mem_a[i*16+j] = {8{4'(i & 4'hF)}};  // 8个相同的INT4值填充PE_DATA_WIDTH
                
                // B矩阵：所有列的值相同 (使用列索引作为INT4值)
                mem_b[i*16+j] = {8{4'(j & 4'hF)}};  // 8个相同的INT4值填充PE_DATA_WIDTH
                
                // C矩阵：全部设为1
                mem_c[i*16+j] = {4{8'h01}};  // 4个INT8值为1
                
                // D矩阵初始化为0
                mem_d[i*16+j] = 0;
            end
        end
        
        // 配置TPU寄存器
        $display("START TPU REG...");
        write_reg(ADDR_MATRIX_SIZE, {11'd16, 11'd16, 10'd16});  // 矩阵尺寸: 16x16x16
        write_reg(ADDR_OP_MODE, {28'b0, 1'b1, 3'b000});        // INT4精度模式，加C矩阵
        write_reg(ADDR_INPUT_A_ADDR, A_BASE_ADDR);             // 矩阵A基地址
        write_reg(ADDR_INPUT_B_ADDR, B_BASE_ADDR);             // 矩阵B基地址
        write_reg(ADDR_INPUT_C_ADDR, C_BASE_ADDR);             // 矩阵C基地址
        write_reg(ADDR_OUTPUT_D_ADDR, D_BASE_ADDR);            // 矩阵D基地址
        
        // 启动计算
        $display("start compute...");
        write_reg(ADDR_CONTROL, 32'h00000001);
        
        // 等待计算完成
        $display("wait compute...");
       
        do begin
            #1000; // 等待一段时间
            read_reg(ADDR_CONTROL, status);
        end while(status[0] == 0);
        
        // 计算完成，验证结果
        $display("finish and test...");
        
        // 计算参考结果
        
        
       
        for (int i = 0; i < 16; i++) begin
            for (int j = 0; j < 16; j++) begin
                // 计算矩阵乘法: D[i,j] = ∑(A[i,k] * B[k,j]) + C[i,j]
                integer sum = 0;
                for (int k = 0; k < 16; k++) begin
                    // 提取INT4值并转换为有符号整数
                    integer a_val = $signed(mem_a[i*16+k][3:0]);
                    integer b_val = $signed(mem_b[k*16+j][3:0]);
                    sum += a_val * b_val;
                end
                
                // 加上C矩阵的值 (INT8)
                sum += $signed(mem_c[i*16+j][7:0]);
                
                // 限制在INT8范围内
                if (sum > 127) sum = 127;
                else if (sum < -128) sum = -128;
                
                expected_d[i*16+j] = sum;
                
                // 比较结果
                 actual = $signed(mem_d[i*16+j][7:0]);
                
                if (actual != expected_d[i*16+j]) begin
                    $display("fail: D[%0d,%0d] = %0d, expected = %0d", 
                             i, j, actual, expected_d[i*16+j]);
                    errors++;
                end
            end
        end
        
        // 显示验证结果
        if (errors == 0)
            $display("OK ALL。");
        else
            $display("FAIL! have %0d mistakes。", errors);
        
        // 结束仿真
        #1000;
        $finish;
    end
endmodule