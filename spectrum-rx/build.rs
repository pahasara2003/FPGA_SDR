fn main() {
    println!("cargo:rustc-link-lib=ftdi1");
    println!("cargo:rustc-link-lib=usb-1.0");
}
