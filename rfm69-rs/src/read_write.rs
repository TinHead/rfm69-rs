use embedded_hal::spi::Operation;
use embedded_hal_async::spi::SpiDevice;

use crate::registers::Register;

pub trait ReadWrite {
    type Error;

    /// Direct write to RFM69 registers.
    fn write_many(
        &mut self,
        reg: Register,
        data: &[u8],
    ) -> impl core::future::Future<Output = core::result::Result<(), Self::Error>>;

    /// Direct read from RFM69 registers.
    fn read_many(
        &mut self,
        reg: Register,
        buffer: &mut [u8],
    ) -> impl core::future::Future<Output = core::result::Result<(), Self::Error>>;
}

impl<S, E> ReadWrite for S
where
    S: SpiDevice<u8, Error = E>,
{
    type Error = E;

    async fn write_many(&mut self, reg: Register, data: &[u8]) -> core::result::Result<(), E> {
        let write = [reg.write()];
        let mut operations = [Operation::Write(&write), Operation::Write(data)];
        self.transaction(&mut operations).await
    }

    async fn read_many(&mut self, reg: Register, buffer: &mut [u8]) -> core::result::Result<(), E> {
        let read = [reg.read()];
        let mut operations = [Operation::Write(&read), Operation::TransferInPlace(buffer)];
        self.transaction(&mut operations).await
    }
}
