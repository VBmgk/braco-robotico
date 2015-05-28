LIBS = -lmat2cpp -llapack
CXXFLAGS = -Wall -std=c++11 -g -O3
SRC = hanoi.cpp
EXEC = hanoi

$(EXEC): $(SRC) ../octave/scripts/kin6dof.h
	$(CXX) $< -o $@ $(CXXFLAGS) $(LIBS)

.PHONY: run
run: $(EXEC)
	./$(EXEC)
