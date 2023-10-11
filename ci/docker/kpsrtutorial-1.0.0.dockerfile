FROM ghcr.io/klepsydra-technologies/kpsrbase:1.0.0

# Install kpsr-sdk
ARG KPSRCOREVERSION="main"
RUN git clone https://github.com/klepsydra-technologies/kpsr-sdk.git && \
    cd kpsr-sdk && \
    git checkout ${KPSRCOREVERSION} && \
    git submodule update --init --recursive && \
    mkdir build && cd build && \
    cmake ../ && \
    make -j$(nproc) && \
    sudo make install && \
    cd ../../ && \
    rm -rf kpsr-sdk

# Install kpsr-core
ARG KPSRCOREVERSION="main"
RUN git clone https://github.com/klepsydra-technologies/kpsr-core.git && \
    cd kpsr-core && \
    git checkout ${KPSRCOREVERSION} && \
    git submodule update --init --recursive && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DKPSR_WITH_DOXYGEN=true -DKPSR_WITH_ZMQ=true -DKPSR_TEST_PERFORMANCE=true -DKPSR_WITH_SOCKET=true -DKPSR_WITH_CODE_METRICS=true ../ && \
    make -j$(nproc) && \
    sudo make install && \
    cd ../../ && \
    rm -rf kpsr-core

# Install Python package build needed by kpsr-codegen
RUN pip3 install build
# Install kpsr-codegen
ARG KPSRCODEGENVERSION="main"
RUN git clone https://github.com/klepsydra-technologies/kpsr-codegen.git && \
    cd kpsr-codegen && \
    git checkout ${KPSRCODEGENVERSION} && \
    python3 -m build --outdir dist && \
    pip3 install dist/kpsr_codegen-1.0-py3-none-any.whl && \
    cd ../../ && \
    rm -rf kpsr-codegen

# Install kpsr-tutorial
ARG KPSRTUTORIALVERSION="main"
RUN git clone https://github.com/klepsydra-technologies/kpsr-tutorial.git && \
    cd kpsr-tutorial && \
    git checkout ${KPSRTUTORIALVERSION} && \
    git submodule update --init --recursive && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release ../ && \
    make -j$(nproc) && \
    sudo make install && \
    cd ../../ && \
    rm -rf kpsr-tutorial
