SUMMARY = "TSCS454 Linux kernel module"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://COPYING;md5=12f884d2ae1ff87c09e5b7ccc2c4ca7e"

inherit module

SRC_URI = "git://www.github.com/Tempo-Semiconductor/tscs454_linux_module.git;rev=master"	

S = "${WORKDIR}/git/"

# The inherit of module.bbclass will automatically name module packages with
# "kernel-module-" prefix as required by the oe-core build environment.

RPROVIDES_${PN} += "kernel-module-tscs454"
