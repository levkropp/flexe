#!/usr/bin/env python3
"""Virtual BLE controller + GATT phone for flexe's --ble-hci backend.

Starts a Bumble virtual controller that the emulator connects to over TCP,
then acts as a phone on the same virtual link: scans for the guest's
advertising, connects, walks the full GATT table (service/characteristic
discovery, reads, writes, notification subscribes), and disconnects.

Usage:
    python3 tools/ble_hci_peer.py [PORT]      # default 9544

Marker lines (BLE_PEER_*) are asserted by scripts/check-s3-ble-hci.sh.
Requires the Bumble package: pip3 install bumble
"""
import asyncio
import logging
import sys

DEFAULT_PORT = 9544


async def main(port):
    try:
        from bumble.controller import Controller
        from bumble.link import LocalLink
        from bumble.device import Device, Peer
        from bumble.host import Host
        from bumble.transport import open_transport
        from bumble import hci
    except ImportError:
        print("ERROR: bumble not installed. Run: pip3 install bumble")
        sys.exit(2)

    link = LocalLink()

    print(f"[*] Starting virtual BLE controller on tcp-server:_:{port}",
          flush=True)
    emu_transport = await open_transport(f"tcp-server:_:{port}")
    emu_controller = Controller(
        "emu-controller",
        host_source=emu_transport.source,
        host_sink=emu_transport.sink,
        link=link,
        public_address="00:11:22:33:44:55",
    )
    emu_controller.random_address = hci.Address(
        "00:11:22:33:44:55", hci.Address.PUBLIC_DEVICE_ADDRESS
    )
    print(f"[*] Waiting for emulator to connect on port {port}...", flush=True)

    phone_controller = Controller(
        "phone-controller",
        link=link,
        public_address="AA:BB:CC:DD:EE:FF",
    )
    phone_host = Host()
    phone_host.controller = phone_controller
    phone_controller.host = phone_host
    phone = Device(name="Test Phone", host=phone_host)
    await phone.power_on()
    print("[*] Phone powered on, waiting for advertising...", flush=True)

    await asyncio.sleep(15)

    devices_found = []

    def on_advertisement(advertisement):
        addr = str(advertisement.address)
        if addr not in [str(d.address) for d in devices_found]:
            devices_found.append(advertisement)
            print(f"[+] Found: {advertisement.address}", flush=True)

    phone.on('advertisement', on_advertisement)
    for round in range(6):
        if devices_found:
            break
        await phone.start_scanning()
        print(f"[*] Scanning round {round} (10s)...", flush=True)
        await asyncio.sleep(10)
        await phone.stop_scanning()

    if not devices_found:
        print("BLE_PEER_NO_DEVICES", flush=True)
        return

    print(f"BLE_PEER_FOUND_{len(devices_found)}", flush=True)
    target = devices_found[0]
    print(f"[*] Connecting to {target.address}...", flush=True)
    try:
        connection = await phone.connect(target.address, timeout=10)
        print("BLE_PEER_CONNECTED", flush=True)
        peer = Peer(connection)
        print("[*] Discovering services...", flush=True)
        await peer.discover_services()
        for service in peer.services:
            print(f"  Service: {service.uuid}", flush=True)
            await service.discover_characteristics()
            for char in service.characteristics:
                props = []
                if char.properties & 0x02:
                    props.append("READ")
                if char.properties & 0x08:
                    props.append("WRITE")
                if char.properties & 0x10:
                    props.append("NOTIFY")
                print(f"    Char: {char.uuid} [{','.join(props)}]", flush=True)
                if char.properties & 0x02:
                    try:
                        value = await char.read_value()
                        print(f"      Value: {value.hex()}", flush=True)
                    except Exception as e:
                        print(f"      Read error: {e}", flush=True)
                if char.properties & 0x08:
                    try:
                        await char.write_value(b'\x01\x02\x03')
                        print(f"    [{char.uuid}] Write OK", flush=True)
                    except Exception as e:
                        print(f"    [{char.uuid}] Write error: {e}", flush=True)
                if char.properties & 0x10:
                    try:
                        await char.discover_descriptors()
                        await char.subscribe()
                        print(f"    [{char.uuid}] Subscribe OK", flush=True)
                    except Exception as e:
                        print(f"    [{char.uuid}] Subscribe error: {e}",
                              flush=True)
        print("BLE_PEER_GATT_DONE", flush=True)
        await asyncio.sleep(1)
        await connection.disconnect()
        print("BLE_PEER_DISCONNECTED", flush=True)
    except Exception as e:
        print(f"[-] Error: {e}", flush=True)
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_PORT
    asyncio.run(main(port))
