PATCH=	box2d_fixed.patch

all:
	(cd Contrib/freeglut; $(MAKE) $(MAKOPTS))
	(cd Contrib/glui; $(MAKE) $(MAKOPTS))
	(cd Source; $(MAKE) $(MAKOPTS))
	(cd Examples/TestBed; $(MAKE) $(MAKOPTS))

clean:
	(cd Contrib/freeglut; $(MAKE) $(MAKOPTS) clean)
	(cd Contrib/glui; $(MAKE) $(MAKOPTS) clean)
	(cd Source; $(MAKE) $(MAKOPTS) clean)
	(cd Examples/TestBed; $(MAKE) $(MAKOPTS) clean)

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
