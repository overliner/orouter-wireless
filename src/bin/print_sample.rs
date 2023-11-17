use std::str::FromStr;

use anyhow::{Context, Result};
use base64::{engine::general_purpose, Engine as _};
use clap::{Parser, Subcommand};

#[derive(Debug, Parser)]
#[command(author, version, about, long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Debug, Subcommand)]
enum Commands {
    /// does testing things
    Slice {
        /// random seed for MessageSlicer
        #[arg(short, long)]
        seed: u64,

        /// input is hex encoded
        #[arg(short, long)]
        input_hex: bool,

        /// desired output encoding, if not provided, Debug impl is used
        #[arg(short, long)]
        output_encoding: Option<OutputEncoding>,

        data: String,
    },
}

#[derive(Debug, Clone)]
enum OutputEncoding {
    Hex,
    Base64,
}

impl FromStr for OutputEncoding {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "hex" => Ok(Self::Hex),
            "b64" | "base64" => Ok(Self::Base64),
            _ => Err(anyhow::anyhow!(
                "{:?} is not a valid encoding specifcation",
                s
            )),
        }
    }
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match &cli.command {
        Commands::Slice {
            seed,
            input_hex,
            output_encoding,
            data,
        } => {
            println!("original data = {:?}", data);
            let data = if *input_hex { todo!() } else { data.as_bytes() };
            let mut slicer = wireless_protocol::MessageSlicer::new(*seed);
            let messages = slicer
                .slice(data, wireless_protocol::MessageType::Data, 0x01)
                .map_err(|e| anyhow::anyhow!("{:?}", e))
                .context("Could not slice message")?;
            println!("slices: \n");
            for (i, message) in messages.iter().enumerate() {
                print_message(i, message, output_encoding.clone())?;
            }
        }
    }
    Ok(())
}

fn print_message(idx: usize, message: &Vec<u8>, encoding: Option<OutputEncoding>) -> Result<()> {
    match encoding {
        None => println!("{}: {:02x?}", idx, message),
        Some(OutputEncoding::Hex) => println!("{}: {}", idx, hex::encode(message)),
        Some(OutputEncoding::Base64) => {
            println!("{}: {}", idx, general_purpose::STANDARD.encode(message))
        }
    };
    Ok(())
}
