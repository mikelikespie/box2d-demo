TARGETS=	Gen/float/testbed #Gen/fixed/testbed

PROJECT=	../..

CXXFLAGS=	-g -O2 -I/usr/lib/include -I$(PROJECT)/Contrib -I$(PROJECT)/Include -DGLUI_FREEGLUT

LDFLAGS=	-L/opt/local/lib 

SOURCES=	Framework/Main.cpp \
		Framework/Test.cpp \
		Framework/Render.cpp \
		Tests/BroadPhaseTest.cpp \
		Tests/Biped.cpp \
		Tests/BipedDef.cpp \
		Tests/TriangleMesh.cpp \
		Tests/TestEntries.cpp 

ifneq ($(INCLUDE_DEPENDENCIES),yes)

all:	
	$(MAKE) --no-print-directory INCLUDE_DEPENDENCIES=yes $(TARGETS)

.PHONY:	clean
clean:
	rm -rf Gen

else

-include $(addprefix Gen/float/,$(SOURCES:.cpp=.d))
-include $(addprefix Gen/fixed/,$(SOURCES:.cpp=.d))

endif



FLOAT_OBJECTS= $(addprefix Gen/float/,$(SOURCES:.cpp=.o))

Test/TopDownCar.h: 

Gen/float/Tests/TestEntries.o: Tests/TestEntries.cpp Test/TopDownCar.h
	mkdir -p $(dir $@)
	c++ $(CXXFLAGS) -c -o $@ $<
	
Gen/float/%.o:		%.cpp
	mkdir -p $(dir $@)
	c++ $(CXXFLAGS) -c -o $@ $<

Gen/float/testbed:	$(FLOAT_OBJECTS) $(PROJECT)/Source/Gen/float/libbox2d.a
	c++ -o $@ $^ $(LDFLAGS) -L$(PROJECT)/Source/Gen/float -lbox2d -lglui -lglut -lGLU -lGL

Gen/float/%.d:		%.cpp
	@mkdir -p $(dir $@)
	c++ -M -MT $(@:.d=.o) $(CXXFLAGS) -o $@ $<



FIXED_OBJECTS= $(addprefix Gen/fixed/,$(SOURCES:.cpp=.o))

Gen/fixed/Tests/TestEntries.o: Tests/TestEntries.cpp Test/TopDownCar.h
	mkdir -p $(dir $@)
	c++ $(CXXFLAGS) -DTARGET_FLOAT32_IS_FIXED -c -o $@ $<
	
Gen/fixed/%.o:		%.cpp
	mkdir -p $(dir $@)
	c++ $(CXXFLAGS) -DTARGET_FLOAT32_IS_FIXED -c -o $@ $<

Gen/fixed/testbed:	$(FIXED_OBJECTS) $(PROJECT)/Source/Gen/fixed/libbox2d.a
	c++ -rdynamic -o $@ $^ $(LDFLAGS) -L$(PROJECT)/Source/Gen/fixed -lbox2d -lglui -lglut -lGLU -lGL

Gen/fixed/%.d:		%.cpp
	@mkdir -p $(dir $@)
	c++ -M -MT $(@:.d=.o) $(CXXFLAGS) -o $@ $<


