FROM ghcr.io/aica-technology/control-libraries/development-dependencies:20.04 as source-dependencies

RUN apt-get update && apt-get install -y libpoco-dev

WORKDIR /source
RUN git clone --depth 1 -b develop https://github.com/aica-technology/control-libraries \
    && cd control-libraries/source && ./install.sh -y --no-controllers --no-dynamical-systems --no-robot-model || exit 1
RUN cd control-libraries/protocol && ./install.sh -y || exit 1
RUN git clone --depth 1 -b develop https://github.com/aica-technology/network-interfaces \
    && cd network-interfaces/source && bash install.sh -y || exit 1
RUN git clone https://github.com/doosan-robotics/API-DRFL \
    && mv API-DRFL/library/Linux/64bits/20.04/libDRFL.a /usr/local/lib \
    && mv API-DRFL/include/* /usr/local/include

WORKDIR ${HOME}
RUN rm -rf /source


FROM source-dependencies as runtime

COPY --chown=${USER} ./source ./
RUN cd doosan_lightweight_interface && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make \
  && make install && ldconfig
WORKDIR ${HOME}
RUN rm -rf ${HOME}/doosan_lightweight_interface
USER ${USER}

ENTRYPOINT /bin/bash
