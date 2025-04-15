`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03.04.2025 23:31:05
// Design Name: 
// Module Name: mbist_final3
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module mbist_final3(
    input clk,
    input reset,
    input bist_mode,
    input [3:0] wraddr_in,
    input [3:0] rdaddr_in,
    input [7:0] data,
    input [1:0] fault,
    output [7:0] data_out,
    output bist_status,
    output [3:0] mux_wraddr,
    output [3:0] mux_rdaddr,
    output [7:0] mux_data_in
);
    wire [3:0] fsm_wraddr, fsm_rdaddr; // FSM generated addresses
    wire [3:0] gen_wraddr, gen_rdaddr; // Address_Generator addresses
    wire [7:0] data_in_gen;
    wire wr_en, rd_en;
    wire [1:0] mode;
    wire [7:0] mem_data_out;

    // Correct Address Selection Using MUX
    assign mux_wraddr = bist_mode ? gen_wraddr : fsm_wraddr;
    assign mux_rdaddr = bist_mode ? gen_rdaddr : fsm_rdaddr;
    assign mux_data_in = bist_mode ? data_in_gen : data;

    Combine3 wrapper (
        .clk(clk),
        .reset(reset),
        .enable(1'b1),
        .test_active(bist_mode),
        .fsm_wraddr(fsm_wraddr),
        .fsm_rdaddr(fsm_rdaddr),
        .gen_wraddr(gen_wraddr),
        .gen_rdaddr(gen_rdaddr),
        .data_in(data_in_gen),
        .mode_gen(mode),
        .wr_en(wr_en),
        .rd_en(rd_en)
    );

    Memory mem (
        .clk(clk),
        .reset(reset),
        .wraddr(mux_wraddr),
        .rdaddr(mux_rdaddr),
        .din(mux_data_in),
        .wr_en(wr_en),
        .rd_en(rd_en),
        .fault(fault),
        .dout(mem_data_out)
    );

    Comparator comp (
        .expected(mux_data_in),
        .actual(mem_data_out),
        .pass_fail(bist_status)
    );

    assign data_out = mem_data_out;
endmodule
module Combine3( 
    input clk,
    input reset,
    input enable,
    input test_active,
    output [7:0] data_in,
    output [3:0] fsm_wraddr,  // Separate FSM write address
    output [3:0] fsm_rdaddr,  // Separate FSM read address
    output [3:0] gen_wraddr,  // Separate Address_Generator write address
    output [3:0] gen_rdaddr,  // Separate Address_Generator read address
    output [1:0] mode_gen,
    output wr_en,
    output rd_en
);
    wire [7:0] generated_data;

    FSM fsm (
        .clk(clk),
        .reset(reset),
        .test_active(test_active),
        .wraddr(fsm_wraddr),
        .rdaddr(fsm_rdaddr),
        .mode_gen(mode_gen),
        .wr_en(wr_en),
        .rd_en(rd_en)
    );

    Address_Generator addr_gen_wr (
        .clk(clk),
        .reset(reset),
        .enable(wr_en),
        .addr_out(gen_wraddr) // Use separate signal
    );

    Address_Generator addr_gen_rd (
        .clk(clk),
        .reset(reset),
        .enable(rd_en),
        .addr_out(gen_rdaddr) // Use separate signal
    );

    Data_Generator data_gen (
        .clk(clk),
        .reset(reset),
        .mode(mode_gen),
        .data_out(generated_data)
    );

    assign data_in = generated_data;
endmodule
// Address Generator Module
module Address_Generator(
    input clk,
    input reset,
    input enable,
    output reg [3:0] addr_out
);

    reg [3:0] binary_count;

    // Binary counter
    always @(posedge clk) begin
        if (reset)
            binary_count <= 4'b0000;
        else if (enable)
            binary_count <= binary_count + 1;
    end

    // Convert binary to Gray code
    always @(*) begin
        addr_out[3] = binary_count[3];
        addr_out[2] = binary_count[3] ^ binary_count[2];
        addr_out[1] = binary_count[2] ^ binary_count[1];
        addr_out[0] = binary_count[1] ^ binary_count[0];
    end

endmodule


// Memory Module
module Memory(
    input clk,
    input reset,
    input [3:0] wraddr,
    input [3:0] rdaddr,
    input [7:0] din,
    input wr_en,
    input rd_en,
    input [1:0] fault,
    output reg [7:0] dout
);
    reg [7:0] mem [15:0]; // 16 x 8-bit memory
    integer i;

    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < 16; i = i + 1)
                mem[i] <= 8'h00; // Initialize memory to 0
            dout <= 8'h00; // Reset output data
        end else begin
            if (wr_en) begin
                case(fault)
                    2'b00: begin  
                   
                        if (wraddr== 4'b0101) begin
                            mem[wraddr] <= 1; // stuck at one fault
                        end 
                        else begin
                             mem[wraddr] <= din;
                        end
                            
                    end
                    2'b01: begin
                          if (wraddr== 4'b1011) begin
                            mem[wraddr] <= 0; // stuck at zero fault
                        end 
                        else begin
                             mem[wraddr] <= din;
                        end
                    end
                    2'b10: begin
                          if (wraddr== 4'b1101) begin
                            mem[wraddr] <= mem[wraddr]&din ; //0->1 transition fault  
                        end 
                        else begin
                             mem[wraddr] <= din;
                        end
                    end
                    2'b11: begin
                          if (wraddr== 4'b1000) begin
                            mem[wraddr] <= mem[wraddr]|din ; // 1->0 transition fault
                        end 
                        else begin
                             mem[wraddr] <= din;
                        end
                    end
                endcase
            end
           else 
            begin 
              if(rd_en)
                dout <= mem[rdaddr]; // Read data from memory
        end
       end
    end
endmodule

// Data Generator Module
module Data_Generator(
    input clk,
    input reset,
    input [1:0] mode,
    output reg [7:0] data_out
);
    always @(posedge clk) begin
        if (reset)
            data_out <= 8'h00; // Reset data output
        else begin
            case (mode)
                2'b00: data_out <= 8'h00; // Mode 0
                2'b01: data_out <= 8'hAA; // Mode 1
                2'b10: data_out <= 8'h55; // Mode 2
                2'b11: data_out <= 8'hFF; // Mode 3
                default: data_out <= 8'h00; // Default case
            endcase
        end
    end
endmodule

// FSM Module
module FSM(
    input clk,
    input reset,
    input test_active,
    output reg [3:0] wraddr,
    output reg [3:0] rdaddr,
    output reg [1:0] mode_gen,
    output reg wr_en,
    output reg rd_en,
    input [7:0] data_out // Data read from the memory
);
    reg [3:0] state; // State register

    // State encoding
    localparam IDLE    = 4'b0000, 
               WRITE1  = 4'b0001, 
               READ1   = 4'b0010,
               WRITE2  = 4'b0011, 
               READ2   = 4'b0100,
               WRITE3  = 4'b0101, 
               READ3   = 4'b0110,
               WRITE4  = 4'b0111, 
               READ4   = 4'b1000;

    always @(posedge clk) begin
        if (reset) begin
            // Reset all registers
            state <= IDLE;
            wr_en <= 0;
            rd_en <= 0;
            wraddr <= 0;
            rdaddr <= 0;
            mode_gen <= 2'b00;
        end else begin
            case (state)
                IDLE: begin
                    if (test_active) begin
                        wr_en <= 1;
                        rd_en <= 0;
                        mode_gen <= 2'b00;
                        wraddr <= 0;
                        rdaddr <= 0;
                        state <= WRITE1;
                    end
                end

                WRITE1: begin
                    if (wraddr < 4'b1111) begin
                        wraddr <= wraddr + 1;
                    end else begin
                        wraddr <= 0;
                        wr_en <= 0;
                        rd_en <= 1;
                        state <= READ1;
                    end
                end

                READ1: begin
                    if (rdaddr < 4'b1111) begin
                        rdaddr <= rdaddr + 1;
                         if (data_out != 8'h00) begin
                         $display("Error in READ1 at addr %d", rdaddr);
                        end
                    end else begin
                        rdaddr <= 0;
                        rd_en <= 0;
                        wr_en <= 1;
                        mode_gen <= 2'b01;
                        state <= WRITE2;
                    end
                end

                WRITE2: begin
                    if (wraddr < 4'b1111) begin
                        wraddr <= wraddr + 1;
                    end else begin
                        wraddr <= 0;
                        wr_en <= 0;
                        rd_en <= 1;
                        state <= READ2;
                    end
                end

                READ2: begin
                    if (rdaddr < 4'b1111) begin
                        rdaddr <= rdaddr + 1;
                        if (data_out != 8'hAA) begin
                            $display("Error in READ2 at addr %d", rdaddr);
                        end
                    end else begin
                        rdaddr <= 0;
                        rd_en <= 0;
                        wr_en <= 1;
                        mode_gen <= 2'b10;
                        state <= WRITE3;
                    end
                end

                WRITE3: begin
                    if (wraddr < 4'b1111) begin
                        wraddr <= wraddr + 1;
                    end else begin
                        wraddr <= 0;
                        wr_en <= 0;
                        rd_en <= 1;
                        state <= READ3;
                    end
                end

                READ3: begin
                    if (rdaddr < 4'b1111) begin
                        rdaddr <= rdaddr + 1;
                        if (data_out != 8'h55) begin
                            $display("Error in READ3 at addr %d", rdaddr);
                        end
                    end else begin
                        rdaddr <= 0;
                        rd_en <= 0;
                        wr_en <= 1;
                        mode_gen <= 2'b11;
                        state <= WRITE4;
                    end
                end

                WRITE4: begin
                    if (wraddr < 4'b1111) begin
                        wraddr <= wraddr + 1;
                    end else begin
                        wraddr <= 0;
                        wr_en <= 0;
                        rd_en <= 1;
                        state <= READ4;
                    end
                end

                READ4: begin
                    if (rdaddr < 4'b1111) begin
                        rdaddr <= rdaddr + 1;
                        if (data_out != 8'hFF) begin
                            $display("Error in READ4 at addr %d", rdaddr);
                        end
                    end else begin
                        rdaddr <= 0;
                        rd_en <= 0;
                        state <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule

// Comparator Module
module Comparator(
    input [7:0] expected,
    input [7:0] actual,
    output pass_fail
);
    assign pass_fail = (expected == actual);
endmodule
