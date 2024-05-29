## Instructions

1. Make sure that you build and release:
    ```bash
    cargo build --release
    ```

2. Make sure that you have `elf2uf2-rs`. To see if it is installed, you can run:
    ```bash
    elf2uf2-rs --help
    ```

3. If it is not installed, you can use this command to install it:
    ```bash
    cargo install elf2uf2-rs
    ```

4. Flash the soft to the Pico:
    ```bash
    elf2uf2-rs.exe -d -s ./target/thumbv6m-none-eabi/release/house_alarm
    ```
