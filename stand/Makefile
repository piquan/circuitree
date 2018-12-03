# This is in a Makefile form so that you can use "make -j8" and
# get parallel builds of several stands.

# If you don't want to use this Makefile, you can make the STL files
# with the shell.  For a stand without a name plaque, use this
# command:
#     openscad -o stand.stl stand.scad
# For a stand with a name plaque:
#     openscad -D 'name="Smith"' -o stand-smith.stl stand.scad

# Set to the names you want to use for customized name plaques.
NAMES := Smith Johnson Williams Taylor

all:

stand.stl: stand.scad
	openscad -o stand.stl stand.scad

define make-stl-target
  $(shell echo stand-$1.stl | tr '[:upper:] ' '[:lower:]-'): stand.scad
	openscad -D "name=\"$1\"" -o $(shell echo stand-$1.stl | tr '[:upper:] ' '[:lower:]-') stand.scad
  all: $(shell echo stand-$1.stl | tr '[:upper:] ' '[:lower:]-')
endef
$(foreach NAME,$(NAMES),$(eval $(call make-stl-target,$(NAME))))

all: stand.stl
