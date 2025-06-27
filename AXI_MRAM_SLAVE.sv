// ========================================
// AXI MRAM Slave (64-bit)
// Author: acelin0121
// This code is for learning and research purposes only.
// It must not be used for any commercial applications.
// All logic in one module version
// ========================================

module axi_mram_slave #(
    parameter ADDR_WIDTH = 10,
    parameter DATA_WIDTH = 64,
    parameter READ_DELAY  = 3,
    parameter TIMEOUT_CYCLE = 10000
)(
    input  wire                      ACLK,
    input  wire                      ARESETN,

    // AXI4 Full Write Address
    input  wire [ADDR_WIDTH-1:0]    S_AXI_AWADDR,
    input  wire [7:0]               S_AXI_AWLEN,
    input  wire [2:0]               S_AXI_AWSIZE,
    input  wire [1:0]               S_AXI_AWBURST,
    input  wire                     S_AXI_AWVALID,
    output reg                      S_AXI_AWREADY,

    // AXI4 Full Write Data
    input  wire [DATA_WIDTH-1:0]    S_AXI_WDATA,
    input  wire                     S_AXI_WVALID,
    input  wire                     S_AXI_WLAST,
    output reg                      S_AXI_WREADY,

    // AXI4 Full Write Response
    output reg  [1:0]               S_AXI_BRESP,
    output reg                      S_AXI_BVALID,
    input  wire                     S_AXI_BREADY,

    // AXI4 Full Read Address
    input  wire [ADDR_WIDTH-1:0]    S_AXI_ARADDR,
    input  wire [7:0]               S_AXI_ARLEN,
    input  wire [2:0]               S_AXI_ARSIZE,
    input  wire [1:0]               S_AXI_ARBURST,
    input  wire                     S_AXI_ARVALID,
    output reg                      S_AXI_ARREADY,

    // AXI4 Full Read Data
    output reg  [DATA_WIDTH-1:0]    S_AXI_RDATA,
    output reg  [1:0]               S_AXI_RRESP,
    output reg                      S_AXI_RVALID,
    output reg                      S_AXI_RLAST,
    input  wire                     S_AXI_RREADY,

    // MRAM interface
    output reg                      MRAM_POWER_ON,
    output reg                      MRAM_CS,
    output reg                      MRAM_WE,
    output reg [31:0]               MRAM_ADDR,
    output reg [63:0]               MRAM_WDATA,
    input  wire [63:0]              MRAM_RDATA,
    input  wire                     MRAM_BUSY,

    // Control register interface
    input  wire [15:0]              CTRL_WRITE_DELAY,

    // Error status output
    output reg  [3:0]               DEBUG_CODE,
    output reg                      ERROR_STATUS
);

    // FSM state types
    typedef enum reg [1:0] {
        W_IDLE, W_SETUP, W_WAIT, W_RESP
    } write_state_t;

    typedef enum reg [1:0] {
        R_IDLE, R_WAIT, R_STREAM
    } read_state_t;

    write_state_t w_state;
    read_state_t  r_state;
    wire fifo_overflow = (read_burst_len > 8'd255); // Overflow detection for prefetch FIFO

    reg [7:0] write_burst_len, write_count;
    reg [15:0] write_delay_counter;
    reg [ADDR_WIDTH-1:0] write_base_addr;

    reg [7:0] read_burst_len, read_count;
    reg [ADDR_WIDTH-1:0] read_base_addr;
    reg [1:0] read_latency_counter;
    reg [DATA_WIDTH-1:0] read_prefetch_fifo [0:255]; // Prefetch FIFO for MRAM read burst
    wire fifo_full  = (fifo_wr_ptr >= 8'd255); // Indicates FIFO overflow risk
    wire fifo_empty = (fifo_rd_ptr >= fifo_wr_ptr); // Indicates no unread data
    reg [7:0] fifo_wr_ptr, fifo_rd_ptr;
    reg [DATA_WIDTH-1:0] read_prefetch_fifo; // FIFO register to hold prefetched MRAM data

    reg [31:0] timeout_counter; // Counter to track MRAM operation latency and detect timeout
    reg ERROR_STATUS; // Flag to indicate a timeout or protocol error
    reg [3:0] DEBUG_CODE; // 4-bit code to indicate the type of error for debug
    wire timeout = (timeout_counter >= TIMEOUT_CYCLE); // Asserted if timeout counter exceeds threshold

    always @(posedge ACLK) begin // Clock-synchronized control block
        if (!ARESETN) begin // Reset all counters and status on reset
            timeout_counter <= 0; // Clear counter when operation is not stalling
            ERROR_STATUS <= 0; // Clear error flag on reset
        end else if ((w_state == W_WAIT || r_state == R_WAIT) && MRAM_BUSY) begin
            timeout_counter <= timeout_counter + 1; // Increment timeout counter while MRAM is busy
        end else begin
            timeout_counter <= 0;
        end
    end

    // ----------------------------
    // Write FSM
    // ----------------------------
    always @(posedge ACLK) begin
        if (!ARESETN) begin
            w_state <= W_IDLE;
            S_AXI_AWREADY <= 1;
            S_AXI_WREADY  <= 0;
            S_AXI_BVALID  <= 0;
            S_AXI_BRESP   <= 2'b00;
            MRAM_POWER_ON <= 0;
            MRAM_CS <= 0;
            MRAM_WE <= 0; // Deassert write enable
            MRAM_ADDR <= 0;
            MRAM_WDATA <= 0;
            write_burst_len <= 0;
            write_count <= 0;
            write_delay_counter <= 0; // Reset write delay wait counter
            write_base_addr <= 0;
        end else begin
            case (w_state) // AXI write state machine
                W_IDLE: if (S_AXI_AWVALID) begin // Wait for AXI master to provide write address
                    S_AXI_AWREADY <= 0;
                    S_AXI_WREADY  <= 1;
                    write_burst_len <= S_AXI_AWLEN;
                    write_count <= 0;
                    write_base_addr <= S_AXI_AWADDR;
                    MRAM_POWER_ON <= 1; // Enable MRAM before issuing commands
                    w_state <= W_SETUP;
                end
                W_SETUP: if (S_AXI_WVALID && !MRAM_BUSY) begin
                    MRAM_CS    <= 1; // Assert chip select to access MRAM
                    MRAM_WE    <= 1; // Write enable active
                    MRAM_ADDR  <= write_base_addr + (write_count << S_AXI_AWSIZE);
                    MRAM_WDATA <= S_AXI_WDATA;
                    write_delay_counter <= 0;
                    w_state <= W_WAIT;
                end
                W_WAIT: if (timeout) begin // Timeout occurred, signal error to AXI master
                    ERROR_STATUS <= 1;
                    DEBUG_CODE <= 4'd4; // 4 = Write timeout error
                    S_AXI_BVALID <= 1;
                    S_AXI_BRESP  <= 2'b10; // AXI SLVERR response
                    MRAM_CS <= 0;
                    MRAM_WE <= 0;
                    w_state <= W_RESP;
                end else begin
                    write_delay_counter <= write_delay_counter + 1;
                    if (write_delay_counter == CTRL_WRITE_DELAY) begin
                        MRAM_CS <= 0;
                        MRAM_WE <= 0;
                        if (S_AXI_WLAST || write_count == write_burst_len) begin
                            S_AXI_WREADY <= 0;
                            S_AXI_BVALID <= 1;
                            S_AXI_BRESP  <= 2'b00; // AXI OKAY response
                            w_state <= W_RESP;
                        end else begin
                            write_count <= write_count + 1;
                            w_state <= W_SETUP;
                        end
                    end
                end
                W_RESP: if (S_AXI_BREADY) begin
                    S_AXI_BVALID <= 0;
                    S_AXI_AWREADY <= 1;
                    w_state <= W_IDLE;
                end
            endcase
        end
    end

    // ----------------------------
    // Read FSM
    // ----------------------------
    always @(posedge ACLK) begin
        if (!ARESETN) begin
            r_state <= R_IDLE;
            S_AXI_ARREADY <= 1;
            S_AXI_RVALID  <= 0;
            S_AXI_RLAST   <= 0;
            MRAM_CS       <= 0;
            MRAM_WE       <= 0;
            MRAM_POWER_ON <= 0;
            read_burst_len <= 0;
            read_count <= 0;
            read_base_addr <= 0;
            read_latency_counter <= 0;
        end else begin
            case (r_state) // AXI read state machine
                R_IDLE: if (S_AXI_ARVALID) begin // Wait for AXI master to provide read address
                    if (S_AXI_ARLEN > 8'd255) begin // Burst length too large for prefetch FIFO
                        ERROR_STATUS <= 1;
                        DEBUG_CODE <= 4'd1; // 1 = Read burst too long
                        S_AXI_RVALID <= 1;
                        S_AXI_RRESP  <= 2'b10; // SLVERR
                        S_AXI_RLAST  <= 1;
                        S_AXI_ARREADY <= 1;
                    end else begin
                    fifo_wr_ptr <= 0;
                    fifo_rd_ptr <= 0;
                    S_AXI_ARREADY <= 0;
                    read_burst_len <= S_AXI_ARLEN;
                    read_count <= 0;
                    read_base_addr <= S_AXI_ARADDR;
                    MRAM_POWER_ON <= 1;
                    MRAM_CS <= 1;
                    MRAM_WE <= 0;
                    MRAM_ADDR <= S_AXI_ARADDR;
                    read_latency_counter <= READ_DELAY; // Set delay before data is ready
                    if (fifo_full) begin // Check for FIFO overflow condition
                        ERROR_STATUS <= 1;
                        DEBUG_CODE <= 4'd2; // 2 = FIFO overflow
                        S_AXI_RVALID <= 1;
                        S_AXI_RRESP  <= 2'b10; // SLVERR on FIFO overflow
                        S_AXI_RLAST  <= 1;
                        r_state <= R_IDLE;
                        MRAM_CS <= 0;
                        S_AXI_ARREADY <= 1;
                    end else begin
                    r_state <= R_WAIT;
                    end
                    end
                end
                R_WAIT: if (timeout) begin
                    ERROR_STATUS <= 1;
                    DEBUG_CODE <= 4'd4; // 4 = Read timeout
                    S_AXI_RVALID <= 1;
                    S_AXI_RRESP  <= 2'b10; // AXI SLVERR response on timeout
                    S_AXI_RLAST  <= 1;
                    r_state <= R_IDLE;
                    MRAM_CS <= 0;
                    S_AXI_ARREADY <= 1;
                end else if (read_latency_counter > 0 && !MRAM_BUSY)
                    read_latency_counter <= read_latency_counter - 1;
                else if (!MRAM_BUSY) begin
                    read_prefetch_fifo[fifo_wr_ptr] <= MRAM_RDATA;
                    fifo_wr_ptr <= fifo_wr_ptr + 1;
                    if (fifo_wr_ptr < read_burst_len) begin
                        MRAM_ADDR <= read_base_addr + ((fifo_wr_ptr + 1) << S_AXI_ARSIZE);
                    end
                    if (fifo_wr_ptr == read_burst_len) begin
                        r_state <= R_STREAM;
                    end
                end
                R_STREAM: if (!S_AXI_RVALID && S_AXI_RREADY) begin
                    S_AXI_RVALID <= 1;
                    S_AXI_RDATA  <= read_prefetch_fifo[fifo_rd_ptr]; // Provide prefetched MRAM data to AXI master
                    S_AXI_RLAST  <= (read_count == read_burst_len);
                    MRAM_ADDR <= read_base_addr + ((read_count + 1) << S_AXI_ARSIZE); // Preload next MRAM address
                    read_count <= read_count + 1;
                    if (fifo_empty) begin // Check for FIFO underflow
                        ERROR_STATUS <= 1;
                        DEBUG_CODE <= 4'd3; // 3 = FIFO underflow
                        S_AXI_RRESP  <= 2'b10;
                        S_AXI_RLAST  <= 1;
                        r_state <= R_IDLE;
                        MRAM_CS <= 0;
                        S_AXI_ARREADY <= 1;
                    end
                    fifo_rd_ptr <= fifo_rd_ptr + 1;
                    if (read_count == read_burst_len) begin
                        r_state <= R_IDLE;
                        MRAM_CS <= 0;
                        S_AXI_ARREADY <= 1;
                    end
                end else if (S_AXI_RVALID && S_AXI_RREADY) begin
                    S_AXI_RVALID <= 0;
                end
            endcase
        end
    end

endmodule
