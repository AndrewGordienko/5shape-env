#include <pybind11/pybind11.h>
#include "add.cpp"

namespace py = pybind11;

PYBIND11_MODULE(cmake_example, m) {
    m.def("drawBox", &drawBox, "Draw a box using Box2D");
}