#!/bin/sh

: ${CORES:=$(nproc 2>/dev/null)}
: ${CORES:=$(sysctl -n hw.ncpu 2>/dev/null)}
: ${CORES:=4}

if [ "$1" = "mingw" ]; then
    SUFFIX=-mingw
    TRIPLE=x86_64-w64-mingw32
    MESON_CPU=x86_64
    MESON_CPU_FAMILY=x86_64
    CONFIG_FLAGS="--host=$TRIPLE"
    CMAKE_FLAGS="-DCMAKE_SYSTEM_NAME=Windows -DCMAKE_C_COMPILER=$TRIPLE-gcc -DCMAKE_CXX_COMPILER=$TRIPLE-g++"
    MINGW=1
fi

set -ex

cd $(dirname $0)

./autogen.sh

mkdir -p build-check-autotools$SUFFIX
cd build-check-autotools$SUFFIX

../configure --prefix=$(pwd)/../install-autotools$SUFFIX $CONFIG_FLAGS
make -j$CORES
rm -rf ../install-autotools$SUFFIX
make -j$CORES install

cd ..
mkdir -p build-check-cmake$SUFFIX
cd build-check-cmake$SUFFIX

rm -rf CMakeCache.txt
cmake \
    -G Ninja  \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=$(pwd)/../install-cmake$SUFFIX \
    -DBUILD_SHARED_LIBS=ON \
    $CMAKE_FLAGS \
    ..

ninja
rm -rf ../install-cmake$SUFFIX
ninja install

cmake -DBUILD_SHARED_LIBS=OFF ..

ninja
ninja install

cd ..
mkdir -p build-check-meson$SUFFIX
cd build-check-meson$SUFFIX

if [ -n "$MINGW" ]; then
    cat <<EOF > cross.txt
[binaries]
c = '$TRIPLE-gcc'
cpp = '$TRIPLE-g++'
ar = '$TRIPLE-ar'
strip = '$TRIPLE-strip'
windres = '$TRIPLE-windres'

[host_machine]
system = 'windows'
cpu_family = '$MESON_CPU_FAMILY'
cpu = '$MESON_CPU'
endian = 'little'
EOF
    MESON_FLAGS="--cross-file cross.txt"
fi

# Explicitly pass --libdir; on Linux, Meson defaults to installing into
# <prefix>/lib/<arch> - see https://github.com/mesonbuild/meson/issues/5925.
rm -rf meson-*
meson setup .. \
    --buildtype release \
    --default-library both \
    --prefix $(pwd)/../install-meson$SUFFIX \
    --libdir $(pwd)/../install-meson$SUFFIX/lib \
    $MESON_FLAGS

ninja
rm -rf ../install-meson$SUFFIX
ninja install

cd ..

cd install-autotools$SUFFIX
find . | sed 's/^..//' | sort | grep -v '\.la$' > ../listing-autotools.txt
cd ../install-cmake$SUFFIX
find . | sed 's/^..//' | sort | grep -v '^lib/cmake' > ../listing-cmake.txt
cd ../install-meson$SUFFIX
find . | sed 's/^..//' | sort > ../listing-meson.txt
cd ..

if [ "$(uname)" = "Darwin" ] && [ -z "$MINGW" ]; then
    # On macOS, cmake installs both libfdk-aac.dylib, libfdk-aac.2.dylib and
    # libfdk-aac.2.0.3.dylib (with the first two being symlinks to the latter),
    # while autotools only installs libfdk-aac.dylib and libfdk-aac.2.dylib.
    cat listing-cmake.txt | grep -v 'lib.*\.\d\.\d\.\d\.dylib' > tmp
    mv tmp listing-cmake.txt
fi

diff -u listing-autotools.txt listing-cmake.txt
diff -u listing-autotools.txt listing-meson.txt

compare_library() {
    lib=$1
    # Check the soname, or equivalent.
    if [ -n "$MINGW" ]; then
        get_dllname() {
            $TRIPLE-dlltool --identify $1
        }
        get_exports() {
            if command -v llvm-readobj > /dev/null; then
                llvm-readobj --coff-exports $1 | grep Name: | awk '{print $2}'
            elif $TRIPLE-objdump -p $1 | grep -q "Ordinal\/Name Pointer"; then
                # GNU objdump
                $TRIPLE-objdump -p $1 | sed -n -e '/Ordinal\/Name Pointer/,/^$/p' | sed '1d;$d' | sed 's/^.*\] *//'
            elif $TRIPLE-objdump -p $1 | grep -q "Export Table"; then
                # llvm-objdump
                $TRIPLE-objdump -p $1 | sed -n '/Export Table:/,$p;' | tail -n +5 | awk '{print $3}'
            else
                echo No suitable tool for inspecting DLL exports found
                exit 1
            fi
        }

        SONAME_AUTOTOOLS=$(get_dllname install-autotools-mingw/lib/$lib.dll.a)
        SONAME_CMAKE=$(get_dllname install-cmake-mingw/lib/$lib.dll.a)
        SONAME_MESON=$(get_dllname install-meson-mingw/lib/$lib.dll.a)
        get_exports install-autotools-mingw/bin/$lib*.dll | sort > exports-autotools
        get_exports install-cmake-mingw/bin/$lib*.dll | sort > exports-cmake
        get_exports install-meson-mingw/bin/$lib*.dll | sort > exports-meson
    elif [ "$(uname)" = "Darwin" ]; then
        get_soname() {
            basename $(otool -D $1 | tail -1)
        }
        get_exports() {
            nm -g -U -j $1
        }
        SONAME_AUTOTOOLS=$(get_soname install-autotools/lib/$lib.dylib)
        SONAME_CMAKE=$(get_soname install-cmake/lib/$lib.dylib)
        SONAME_MESON=$(get_soname install-meson/lib/$lib.dylib)
        get_exports install-autotools/lib/$lib.dylib | sort > exports-autotools
        get_exports install-cmake/lib/$lib.dylib | sort > exports-cmake
        get_exports install-meson/lib/$lib.dylib | sort > exports-meson
    else
        get_soname() {
            readelf -d $1 | grep SONAME | sed -e 's/.*soname: //' -e 's/^\[//' -e 's/\]$//'
        }
        get_exports() {
            nm -g -U -j $1
        }
        SONAME_AUTOTOOLS=$(get_soname install-autotools/lib/$lib.so)
        SONAME_CMAKE=$(get_soname install-cmake/lib/$lib.so)
        SONAME_MESON=$(get_soname install-meson/lib/$lib.so)
        get_exports install-autotools/lib/$lib.so | sort > exports-autotools
        get_exports install-cmake/lib/$lib.so | sort > exports-cmake
        get_exports install-meson/lib/$lib.so | sort > exports-meson
    fi
    
    if [ "$SONAME_AUTOTOOLS" != "$SONAME_CMAKE" ]; then
        echo For $lib, autotools SONAME $SONAME_AUTOTOOLS differs from CMake $SONAME_CMAKE
        exit 1
    fi
    if [ "$SONAME_AUTOTOOLS" != "$SONAME_MESON" ]; then
        echo For $lib, autotools SONAME $SONAME_AUTOTOOLS differs from Meson $SONAME_MESON
        exit 1
    fi
    if ! diff -u exports-autotools exports-cmake; then
        echo For $lib, autotools and CMake exports differ
        exit 1
    fi
    if ! diff -u exports-autotools exports-meson; then
        echo For $lib, autotools and Meson exports differ
        exit 1
    fi
}

compare_library libfdk-aac
