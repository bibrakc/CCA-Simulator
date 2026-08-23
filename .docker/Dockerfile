# CCA-Simulator CI container
# Pre-installs all tools needed by GitHub Actions CI to avoid
# repeated installation on every workflow run.
#
# Tools included:
#   - GCC 13 (C++20 with OpenMP)
#   - CMake 3.20+
#   - libomp-dev
#   - clang-format-21 (from LLVM apt repo)
#   - Racket 9.3 minimal (for the CCA language compiler)
#   - rackunit-lib (Racket test framework)
#
# Build:
#   docker build -t ghcr.io/bibrakc/cca-simulator-ci:latest .docker/
#
# Push:
#   docker push ghcr.io/bibrakc/cca-simulator-ci:latest

FROM ubuntu:24.04

LABEL org.opencontainers.image.source="https://github.com/bibrakc/CCA-Simulator"
LABEL org.opencontainers.image.description="CI image for CCA-Simulator with GCC 13, CMake, clang-format-21, and Racket 9.3"

ENV DEBIAN_FRONTEND=noninteractive

# ─── System packages: GCC 13, CMake, OpenMP ──────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc-13 \
    g++-13 \
    cmake \
    make \
    libomp-dev \
    wget \
    ca-certificates \
    gnupg \
    git \
    && rm -rf /var/lib/apt/lists/*

# Set GCC 13 as default
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 100 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 100 && \
    update-alternatives --install /usr/bin/cc cc /usr/bin/gcc-13 100 && \
    update-alternatives --install /usr/bin/c++ c++ /usr/bin/g++-13 100

# ─── clang-format-21 from LLVM apt repository ────────────────────────────────
RUN wget -qO- https://apt.llvm.org/llvm-snapshot.gpg.key | tee /etc/apt/trusted.gpg.d/apt.llvm.org.asc > /dev/null && \
    echo "deb http://apt.llvm.org/noble/ llvm-toolchain-noble-21 main" > /etc/apt/sources.list.d/llvm-21.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends clang-format-21 && \
    rm -rf /var/lib/apt/lists/*

# ─── Racket 9.3 minimal ──────────────────────────────────────────────────────
RUN wget -q https://download.racket-lang.org/installers/9.3/racket-minimal-9.3-x86_64-linux-cs.sh -O /tmp/racket-install.sh && \
    chmod +x /tmp/racket-install.sh && \
    /tmp/racket-install.sh --in-place --dest /opt/racket && \
    rm /tmp/racket-install.sh

ENV PATH="/opt/racket/bin:${PATH}"

# ─── Install rackunit-lib ─────────────────────────────────────────────────────
RUN raco pkg install --auto --batch --scope installation rackunit-lib

# ─── Verify installations ────────────────────────────────────────────────────
RUN gcc --version | head -1 && \
    cmake --version | head -1 && \
    clang-format-21 --version && \
    racket --version && \
    echo "All tools installed successfully"

# Default working directory
WORKDIR /workspace
