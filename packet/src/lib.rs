// https://kpn-iot.github.io/senml-library/
// https://datatracker.ietf.org/doc/html/draft-ietf-core-senml-13

use hex::FromHex;
use minicbor::data::Token;
use minicbor::{encode, Decoder, Encoder};

struct Packet {}

fn encode_temp<'e, W: minicbor::encode::Write>(
    temp: f32,
    encoder: &mut Encoder<W>,
) -> Result<(), minicbor::encode::Error<W::Error>> {
    encoder.begin_map()?.i8(-24)?.f32(temp)?.null()?.end()?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn senml_decode() {
        let input = b"85a300686c6174697475646502fb4049f9ed288ce70401636c6174a300696c6f6e67697475646502fb40155165d3996fa801636c6f6ea3006b74656d706572617475726502fb403dc00000000000016343656ca3006e62617474657279566f6c7461676502fb400cb851eb851eb8016156a3006b74656d706572617475726502fb40313851eb851eb8016343656c".to_vec();
        let senml_bytes: Vec<u8> = FromHex::from_hex::<Vec<u8>>(input).unwrap();
        let tokens = Decoder::new(&senml_bytes)
            .tokens()
            .collect::<Result<Vec<Token>, _>>()
            .unwrap();

        println!("{tokens:?}");
    }

    #[test]
    fn thinkml_decode() {
        let input = b"85a202fb4049f9ed288ce7041735a202fb40155165d3996fa81734a202fb403dc000000000001737a202fb400cb851eb851eb81706a202fb40313851eb851eb81737".to_vec();
        let senml_bytes: Vec<u8> = FromHex::from_hex::<Vec<u8>>(input).unwrap();
        let tokens = Decoder::new(&senml_bytes)
            .tokens()
            .collect::<Result<Vec<Token>, _>>()
            .unwrap();

        println!("{tokens:?}");
    }

    #[test]
    fn temp_encode() {
        let mut buffer = [0u8; 128];
        let mut encoder = Encoder::new(&mut buffer[..]);

        let temp: f32 = 25.7;

        encode_temp(temp, &mut encoder).unwrap();

        println!("{encoder:?}");
    }
}
