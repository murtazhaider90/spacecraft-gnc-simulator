module order_book_top (
    input  wire        clk,
    input  wire        rst,
    input  wire        event_valid,
    input  wire        is_bid,
    input  wire [31:0] price,
    output reg  [31:0] best_bid,
    output reg  [31:0] best_ask,
    output wire [31:0] spread,
    output wire [31:0] mid_price
);

assign spread = (best_ask >= best_bid) ? (best_ask - best_bid) : 32'd0;
assign mid_price = (best_bid + best_ask) >> 1;

always @(posedge clk) begin
    if (rst) begin
        best_bid <= 32'd0;
        best_ask <= 32'hFFFFFFFF;
    end else if (event_valid) begin
        if (is_bid) begin
            if (price > best_bid)
                best_bid <= price;
        end else begin
            if (price < best_ask)
                best_ask <= price;
        end
    end
end

endmodule
