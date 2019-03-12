# Program-specfic section, the target program's name along with its source and object files are listed.
#These are used in the Build Rules section to layer dependency, with TARGET at the top depending on OBJ #which in turn depends on SRC.
# Program-specific
APPNAME := bin/app
SOURCES := $(wildcard src/*.cpp src/*.cu src/*.c)
OBJECTS := $(patsubst src%,obj%, $(patsubst %.cu,%.device.o, $(patsubst %.cpp,%.o, $(patsubst %.c,%.o, $(SOURCES)))))

# Paths section, the paths to the CUDA includes and libraries files are listed. Libraries are used later by g++ while NVCC uses the includes.
#It may be necessary to add additional library declarations to the LIBS. Libraries used must be listed here along with the path to find them. 			
# Paths
INCLUDE := -I/usr/include -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -I/usr/local/cuda-5.5/include -I/usr/include/teem
LIBPATH := -L/usr/local/cuda-5.5/lib64 -L/usr/local/lib -L/usr/local/lib/Teem-1.12.0 -L/usr/lib
#LIBS    := -lcudart 
#LIBS	+=-lpcl_gpu_containers -lpcl_gpu_features -lpcl_gpu_octree -lpcl_gpu_surface -lpcl_gpu_utils 

LIBS    += -lboost_system -lpcl_common -lpcl_io_ply -lpcl_io -lpcl_kdtree -lpcl_keypoints -lpcl_octree -lGL -lGLU -lfltk -lfltk_gl -lteem
# Flags section, the various flags are listed. In the sample program, only NVCCFLAGS are used, the others remain for reference.
# Flags
FLAGS    := -g -Wno-deprecated -fPIC -O2 -ffast-math 
CCFLAGS  := $(FLAGS)
CXXFLAGS := $(FLAGS) 
GENCODE_FLAGS := -gencode arch=compute_20,code=sm_20 -m64
NVCCFLAGS := $(GENCODE_FLAGS) --compiler-options -fno-strict-aliasing -lineinfo -use_fast_math -Xptxas -dlcm=cg 
#-dlink 
#-rdc=true



#In the Compilers section, the paths to the system's compilers are listed.
# Compilers
CC   := gcc
CXX  := g++
NVCC := /usr/local/cuda-5.5/bin/nvcc

# Build Rules section, the makefile's rules are defined.
#Cuda uses nvcc to compile .cu source files into object files.
#g++ is used to link the object files with the needed libraries in the final binary.
# Build Rules
all: $(OBJECTS)
	$(CXX) $(CXXFLAGS) $(INCLUDE) $(OBJECTS) -o $(APPNAME) $(LIBPATH) $(LIBS)

%.o: ../src/%.c
	$(CC) $(CCFLAGS) $(INCLUDE) -c $< -o $@

%.o: ../src/%.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE) -c $< -o $@

%.device.o: ../src/%.cu
	$(NVCC) $(NVCCFLAGS) $(INCLUDE) -c $< -o $@
  
clean:
	rm -rf obj/*
	rm -f $(APPNAME)
	rm -rf src/*~
	rm -rf *~

run:
	$(APPNAME)

