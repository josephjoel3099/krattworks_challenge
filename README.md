# Krattworks Tech Challenge
A simple UDP client-server drone application with a simulated drone client, ground control staion server and gui to control the available services.

## Tested Platform
- Ubuntu 22.04 (WSL2 on Windows 11)

## Prerequisites
- C++ 20
- g++ >= 11.4.0

## Submodules
- [SimpleUDP](https://github.com/RedFox20/SimpleUDP.git)
- [GLFW](https://github.com/glfw/glfw.git)
- [ImGui](https://github.com/ocornut/imgui.git)
- [MAVLink C Library v2](https://github.com/mavlink/c_library_v2.git)
- [Google Test](https://github.com/google/googletest.git)

### Pull submodules
```

```

## Build Instructions
```
rm -rf build
mkdir build
cd build

cmake .. -DCMAKE_CXX_COMPILER=g++
cmake --build .
```

## License
MIT License

Copyright (c) 2026 Joseph Joel

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
