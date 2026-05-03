use embedded_hal::spi::{Operation, SpiDevice};
use embedded_hal_async::spi::SpiDevice as AsyncSpiDevice;

use crate::registers::Register;

pub trait ReadWrite {
    type Error;

    /// Direct write to RFM69 registers.
    fn write_many(&mut self, reg: Register, data: &[u8]) -> core::result::Result<(), Self::Error>;

    /// Direct read from RFM69 registers.
    fn read_many(
        &mut self,
        reg: Register,
        buffer: &mut [u8],
    ) -> core::result::Result<(), Self::Error>;
}

impl<S, E> ReadWrite for S
where
    S: SpiDevice<u8, Error = E>,
{
    type Error = E;

    fn write_many(&mut self, reg: Register, data: &[u8]) -> core::result::Result<(), E> {
        let write = [reg.write()];
        let mut operations = [Operation::Write(&write), Operation::Write(data)];
        self.transaction(&mut operations)
    }

    fn read_many(&mut self, reg: Register, buffer: &mut [u8]) -> core::result::Result<(), E> {
        let read = [reg.read()];
        let mut operations = [Operation::Write(&read), Operation::TransferInPlace(buffer)];
        self.transaction(&mut operations)
    }
}

pub trait AsyncReadWrite {
    type Error;

    async fn write_many_async(&mut self, reg: Register, data: &[u8]) -> core::result::Result<(), Self::Error>;
    async fn read_many_async(&mut self, reg: Register, buffer: &mut [u8]) -> core::result::Result<(), Self::Error>;
}

impl<S, E> AsyncReadWrite for S
where
    S: AsyncSpiDevice<u8, Error = E>,
{
    type Error = E;

    async fn write_many_async(&mut self, reg: Register, data: &[u8]) -> core::result::Result<(), E> {
        use embedded_hal_async::spi::Operation as AsyncOp;
        let write = [reg.write()];
        let mut operations = [AsyncOp::Write(&write), AsyncOp::Write(data)];
        self.transaction(&mut operations).await
    }

    async fn read_many_async(&mut self, reg: Register, buffer: &mut [u8]) -> core::result::Result<(), E> {
        use embedded_hal_async::spi::Operation as AsyncOp;
        let read = [reg.read()];
        let mut operations = [AsyncOp::Write(&read), AsyncOp::TransferInPlace(buffer)];
        self.transaction(&mut operations).await
    }
}
