//@RndMnkIII 02/09/2025
// JVS_pkg.sv
package JVS_pkg;
    //Parameters
    parameter int unsigned MAX_JVS_NODES  = 8;
    parameter int unsigned NODE_NAME_SIZE = 64;

    typedef struct {
    logic [7:0] name   [NODE_NAME_SIZE]; // bytes ASCII, fixed length
    logic [7:0] cmd_ver;                 // Command version
    logic [7:0] jvs_ver;                 // JVS protocol version
    logic [7:0] com_ver;                 // Communication version
    logic [7:0] addr;                    // Node address (node count)
    } jvs_node_t;

    // Node array with MAX_JVS_NODES elements
    typedef jvs_node_t jvs_nodes_t [MAX_JVS_NODES];

endpackage : JVS_pkg