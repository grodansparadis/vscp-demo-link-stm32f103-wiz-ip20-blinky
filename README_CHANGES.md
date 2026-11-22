# Changes in vscp-demo-link-stm32f103-wiz-ip20-blinky

Every change that goes into a release is listed here, newest release first.
Add each change under **Unreleased** when it is made. When a release is
tagged, rename **Unreleased** to the version and date, and start a new
**Unreleased** section above it.

## Unreleased

- Fixed link errors: implemented missing `vscp_link_callback_disconnect`, renamed `vscp_link_callback_clrAll` to `vscp_link_callback_clrall`, and replaced the undefined `sendCommand` with `wiznet_ip20_send_command` (which now also fills the caller's response buffer)
- Reduced flash usage to fit the 32 KB STM32F103C6T6: replaced `rand()` with a local xorshift PRNG, replaced `printf` with the no-op `LOGSTR` in callbacks, added small `strstr`/`strcmp` replacements, and compile HAL sources with `-Os` in Debug builds
- Corrected MCU designation in README (STM32F103C6T6, not C8T6)
- Documented the 32 KB flash memory constraints and size-saving measures in README and docs/build-a-vscp-tcp-server.md
- Removed duplicate `callbacks-link.c` entry in firmware CMakeLists
- Added defines for WIZnet IP20 (58d4058)
- Added initial MDF (83fac2b)
- Added persistent storage (c27e12c)
- Added third-part folder for VSCP firmware repo. files (f530918)
- Initial MDF (4ab92a9)
- Initial MDF (b671ef1)
- Initial connect/disconnect/command consume (6469985)
- Added initial state machine (5ab559a)
- Changed license to MIT (976bd56)
- Added reference schematics (d3e0e59)
- Initial commit (1978c55)

No release has been tagged yet.
