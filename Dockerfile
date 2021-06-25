#FROM roscore/heroehs_vhsc_test:t4_good
#FROM roscore/heroehs_vhsc_test:ti_good
FROM roscore/heroehs_vhsc_test:20.0

USER alice3

# 포트 지정
EXPOSE 80

# TCP Connections
EXPOSE 10001 10002 10003 10004 10021 10022 10023 10024

# UDP Connections for Game Controller
EXPOSE 3838/udp
EXPOSE 3939/udp

# UDP Connections for Other Robot
EXPOSE 3839/udp

# Auto Start
COPY robot_start.sh /
RUN sudo chmod +X /robot_start.sh

ENTRYPOINT ["/bin/bash", "/robot_start.sh"]
