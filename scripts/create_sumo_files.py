from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
MAPS = BASE / "maps"
MAPS.mkdir(exist_ok=True)

net_file = MAPS / "highway.net.xml"
route_file = MAPS / "routes.rou.xml"
config_file = MAPS / "config.sumocfg"

net_xml = """<net version="1.16">
    <location netOffset="0.00,0.00" convBoundary="0.00,0.00,2000.00,0.00" origBoundary="0.00,0.00,2000.00,0.00" projParameter="!"/>

    <edge id="E0" from="J0" to="J1" priority="1">
        <lane id="E0_0" index="0" speed="33.33" length="2000.00" shape="0.00,0.00 2000.00,0.00"/>
        <lane id="E0_1" index="1" speed="33.33" length="2000.00" shape="0.00,3.20 2000.00,3.20"/>
        <lane id="E0_2" index="2" speed="33.33" length="2000.00" shape="0.00,6.40 2000.00,6.40"/>
    </edge>

    <junction id="J0" type="dead_end" x="0.00" y="3.20" incLanes="" intLanes="" shape="0.00,-1.60 0.00,8.00"/>
    <junction id="J1" type="dead_end" x="2000.00" y="3.20" incLanes="E0_0 E0_1 E0_2" intLanes="" shape="2000.00,-1.60 2000.00,8.00"/>
</net>
"""

route_xml = """<routes>
    <vType id="car" accel="2.6" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="30"/>

    <route id="r0" edges="E0"/>

    <flow id="dense_flow"
          type="car"
          route="r0"
          begin="0"
          end="300"
          period="0.35"
          departLane="random"
          departSpeed="random"/>
</routes>
"""

config_xml = """<configuration>
    <input>
        <net-file value="highway.net.xml"/>
        <route-files value="routes.rou.xml"/>
    </input>

    <time>
        <begin value="0"/>
        <end value="300"/>
        <step-length value="0.1"/>
    </time>

    <processing>
        <collision.action value="warn"/>
        <ignore-route-errors value="true"/>
    </processing>
</configuration>
"""

net_file.write_text(net_xml)
route_file.write_text(route_xml)
config_file.write_text(config_xml)

print("SUMO files created in maps/")