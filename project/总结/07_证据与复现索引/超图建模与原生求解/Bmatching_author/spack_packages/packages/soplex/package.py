from spack import *


class Soplex(CMakePackage):
    """Soplex is an optimization library for solving linear programming problems."""

    homepage = "https://soplex.zib.de/"
    url = "https://soplex.zib.de/download/release/soplex-6.0.3.tgz"

    version(
        "6.0.3",
        sha256="357db78d18d9f1b81a4fe65cf0182475d2a527a6ad9030ac2f5ef7c077d6764c",
    )

    depends_on("gmp")
    depends_on("zlib")

    def cmake_args(self):
        args = [
            "-DBUILD_SHARED_LIBS=ON",  # Build shared libraries (optional)
            "-DWITH_GMP=ON",
            "-DGMP_DIR={0}".format(self.spec["gmp"].prefix),  # Path to GMP installation
            "-DWITH_ZLIB=ON",
            "-DBOOST=off",
            "-DZLIB_DIR={0}".format(
                self.spec["zlib"].prefix
            ),  # Path to zlib installation
        ]
        return args
