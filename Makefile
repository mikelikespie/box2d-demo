PATCH=	box2d_fixed.patch

all: testbed lib

testbed:
	(cd Examples/TestBed; $(MAKE) $(MFLAGS))

lib:
	(cd Source; $(MAKE) $(MFLAGS))



clean:
	(cd Source; $(MAKE) clean $(MFLAGS))
	(cd Examples/TestBed; $(MAKE) clean $(MFLAGS))

patch:
	svn diff > $(PATCH)
#	- diff --unified /dev/null Makefile >> $(PATCH)
#	- diff --unified /dev/null Documentation/latex/Makefile >> $(PATCH)
#	- diff --unified /dev/null Contrib/freeglut/Makefile >> $(PATCH)
#	- diff --unified /dev/null Contrib/glui/Makefile >> $(PATCH)
#	- diff --unified /dev/null Source/Makefile >> $(PATCH)
#	- diff --unified /dev/null Examples/TestBed/Makefile >> $(PATCH)
#	- diff --unified /dev/null Source/Common/Fixed.h >> $(PATCH)
#	- diff --unified /dev/null Source/Common/jtypes.h >> $(PATCH)
#	- diff --unified /dev/null Source/Contrib/b2Polygon.cpp >> $(PATCH)
#	- diff --unified /dev/null Source/Contrib/b2Polygon.h >> $(PATCH)
#	- diff --unified /dev/null Source/Contrib/b2Triangle.cpp >> $(PATCH)
#	- diff --unified /dev/null Source/Contrib/b2Triangle.h >> $(PATCH)
