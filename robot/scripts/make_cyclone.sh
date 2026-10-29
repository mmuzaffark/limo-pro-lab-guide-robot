#!/bin/bash
# usage: make_cyclone.sh [LAPTOP_IP]
[ -n "$1" ] && echo "$1" > ~/guide_ws/laptop_ip
P=$(cat ~/guide_ws/laptop_ip 2>/dev/null)
PEER=""; [ -n "$P" ] && PEER="<Peers><Peer address=\"$P\"/></Peers>"
cat > ~/guide_ws/cyclone.xml <<XML
<CycloneDDS><Domain id="any">
<General><NetworkInterfaceAddress>wlan0</NetworkInterfaceAddress></General>
<Discovery>$PEER<ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery>
</Domain></CycloneDDS>
XML
echo "cyclone.xml written (laptop peer: ${P:-none})"
