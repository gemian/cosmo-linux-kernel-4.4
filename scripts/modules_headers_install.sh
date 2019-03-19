#!/bin/sh

# modules_headers_install.sh
#
# Simple script to extract linux headers and friends needed to build
# out of tree modules without having the all source tree around.
#
# Inspired from scripts/package/builddeb

HDR_SRC_FILES=$objtree/hdrsrcfiles
HDR_OBJ_FILES=$objtree/hdrobjfiles
DEST_DIR=linux-modules-headers

if [ -n "$INSTALL_MODULES_HDR_PATH" ]; then
	DEST_DIR="$INSTALL_MODULES_HDR_PATH"
fi

echo sources: $INSTALL_MODULES_HDR_PATH

#fresh start
rm -f $HDR_SRC_FILES
rm -f $HDR_OBJ_FILES

#build list of headers and friends
(cd $srctree; find . -name Makefile\* -o -name Kconfig\* -o -name \*.pl) \
	> "$HDR_SRC_FILES"
(cd $srctree; find arch/$SRCARCH/include include scripts -type f) \
	>> "$HDR_SRC_FILES"
#special case for arm64 headers referencing arm headers
if [ "$SRCARCH" = "arm64" ]; then
(cd $srctree; find arch/arm/include -name 'opcodes.h') \
        >> "$HDR_SRC_FILES"
(cd $srctree; find arch/arm/include -name xen -type d) \
        >> "$HDR_SRC_FILES"
fi
(cd $srctree; find arch/$SRCARCH -name module.lds -o -name Kbuild.platforms \
	-o -name Platform) >> "$HDR_SRC_FILES"
(cd $srctree; find $(find arch/$SRCARCH -name include -o -name scripts \
	-type d) -type f) >> "$HDR_SRC_FILES"
(cd $objtree; find arch/$SRCARCH/include Module.symvers include scripts \
	-type f) >> "$HDR_OBJ_FILES"

mkdir -p "$DEST_DIR"

(cd $srctree; tar -c -f - -T -) < "$HDR_SRC_FILES" | (cd $DEST_DIR; tar -xf -)
(cd $objtree; tar -c -f - -T -) < "$HDR_OBJ_FILES" | (cd $DEST_DIR; tar -xf -)

# copy .config manually to be where it's expected to be
(cd $objtree; cp $KCONFIG_CONFIG $DEST_DIR/.config)