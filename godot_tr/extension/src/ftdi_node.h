#ifndef FTDI_NODE_H
#define FTDI_NODE_H

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>

#include <vector>
#include <ftdi.h>

using namespace godot;

class FtdiNode : public Node {
    GDCLASS(FtdiNode, Node);

    private:
        struct ftdi_context *ftdic = nullptr;
        void close();

    protected:
        static void _bind_methods();

    public:
        int status = 0;
        FtdiNode();
        ~FtdiNode();

        int init(int INTERFACE, int VENDOR, int PRODUCT, int BAUDRATE);
        int write(int c);
        PackedByteArray read(int size);
        int isOK();



        //void _ready() override;

        //void _process(double delta) override;


};


#endif
