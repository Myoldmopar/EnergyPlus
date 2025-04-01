import gdb

class Array3DPrinter:
    """Pretty-print an ObjexxFCL::Array3D<float> instance"""

    def __init__(self, val):
        self.val = val  # Instance of ObjexxFCL::Array3D<float>
        self.data_ptr = val["data_"]  # Pointer to the float array

        # Read dimensions from member variables
        self.l1 = int(val["I1_"]["l_"])
        self.u1 = int(val["I1_"]["u_"])
        self.l2 = int(val["I2_"]["l_"])
        self.u2 = int(val["I2_"]["u_"])
        self.l3 = int(val["I3_"]["l_"])
        self.u3 = int(val["I3_"]["u_"])

        # Compute sizes
        self.x_dim = self.u1 - self.l1 + 1
        self.y_dim = self.u2 - self.l2 + 1
        self.z_dim = self.u3 - self.l3 + 1
        self.total_size = self.x_dim * self.y_dim * self.z_dim

    def to_string(self):
        return f"ObjexxFCL::Array3D<float> [{self.x_dim} x {self.y_dim} x {self.z_dim}] at {self.data_ptr}"

    def num_children(self):
        """CLion should see that this object has `z_dim` children (each layer)."""
        return self.z_dim

    def children(self):
        """Group elements by layers and replace `0.0` values with `_` for clarity"""
        for z in range(self.z_dim):
            layer_values = []
            for y in range(self.y_dim):
                row_values = []
                for x in range(self.x_dim):
                    index = z * (self.y_dim * self.x_dim) + y * self.x_dim + x
                    value = float((self.data_ptr + index).dereference())

                    # Replace 0.0 with "_" to highlight nonzero values
                    formatted_value = "_" if value == 0.0 else f"{value}"
                    row_values.append(formatted_value)

                # Convert row to string so CLion renders it properly
                layer_values.append("[" + ", ".join(row_values) + "]")

            # Convert full layer to a string
            yield f"Layer {z + self.l3}", "[" + ", ".join(layer_values) + "]"

def lookup_type(val):
    if str(val.type).startswith("ObjexxFCL::Array3D<float>"):
        return Array3DPrinter(val)
    return None

# Register the pretty printer
gdb.pretty_printers.append(lookup_type)
