#![allow(dead_code)]

pub struct Buffer<'a, T: 'a> {
    buffer: &'a mut [T],
    read_index: usize,
    write_index: usize,
}

impl<'a, T> Buffer<'a, T>
where
    T: Copy,
{
    pub fn new(buffer: &'a mut [T]) -> Buffer<T> {
        Buffer::<T> {
            buffer,
            read_index: 0,
            write_index: 0,
        }
    }

    pub fn size(&self) -> usize {
        if self.write_index >= self.read_index {
            self.write_index - self.read_index
        } else {
            self.write_index + self.buffer.len() - self.read_index
        }
    }

    pub fn next_contiguous_slice_len(&self) -> usize {
        if self.read_index == 0 {
            self.buffer.len() - self.write_index - 1
        } else if self.write_index >= self.read_index {
            self.buffer.len() - self.write_index
        } else {
            self.read_index - self.write_index - 1
        }
    }

    pub fn next_mut_slice(&mut self, n: usize) -> &mut [T] {
        if n > self.next_contiguous_slice_len() {
            panic!(
                "Not enough contiguous data to write into (wanted {}, have {})",
                n,
                self.next_contiguous_slice_len()
            );
        }

        let start = self.write_index;
        self.write_index = (self.write_index + n) % self.buffer.len();
        &mut self.buffer[start..start + n]
    }

    pub fn available_len(&self) -> usize {
        if self.read_index <= self.write_index {
            self.read_index + self.buffer.len() - self.write_index - 1
        } else {
            self.read_index - self.write_index - 1
        }
    }

    pub fn peek(&self, n: usize) -> T {
        if n >= self.size() {
            panic!("Peek out of range: {} requested, max {}", n, self.size());
        }
        self.buffer[(self.read_index + n) % self.buffer.len()]
    }

    pub fn take_slice<'b>(&mut self, n: usize, buf: &'b mut [T]) {
        if n > self.size() {
            panic!(
                "Not enough data to read (wanted {}, have {})",
                n,
                self.size()
            );
        }
        for (i, byte) in buf.iter_mut().enumerate().take(n) {
            *byte = self.buffer[(self.read_index + i) % self.buffer.len()];
        }
        self.read_index = (self.read_index + n) % self.buffer.len();
    }
}

mod tests {
    #[test]
    fn empty_capacity() {
        const CAPACITY: usize = 8;
        let mut buf: [u8; CAPACITY] = [0; CAPACITY];
        let cbuf = super::Buffer::<u8>::new(&mut buf);
        assert_eq!(cbuf.available_len(), CAPACITY - 1);
        assert_eq!(cbuf.next_contiguous_slice_len(), CAPACITY - 1);
    }

    #[test]
    fn empty_capacity_after_use() {
        const CAPACITY: usize = 8;
        let mut buf: [u8; CAPACITY] = [0; CAPACITY];
        let mut cbuf = super::Buffer::<u8>::new(&mut buf);
        const TRANSFER_SIZE: usize = 4;
        {
            let writable = cbuf.next_mut_slice(TRANSFER_SIZE);
            for i in 0..TRANSFER_SIZE {
                writable[i] = 1 + i as u8;
            }
        }
        assert_eq!(cbuf.available_len(), CAPACITY - TRANSFER_SIZE - 1);
        assert_eq!(
            cbuf.next_contiguous_slice_len(),
            CAPACITY - TRANSFER_SIZE - 1
        );
        assert_eq!(cbuf.peek(0), 1);
        assert_eq!(cbuf.peek(1), 2);
        assert_eq!(cbuf.peek(2), 3);
        assert_eq!(cbuf.peek(3), 4);
        {
            let mut read_from: [u8; TRANSFER_SIZE] = [0; TRANSFER_SIZE];
            cbuf.take_slice(TRANSFER_SIZE, &mut read_from);
            for i in 0..TRANSFER_SIZE {
                assert_eq!(read_from[i], 1 + i as u8);
            }
        }
        assert_eq!(cbuf.available_len(), CAPACITY - 1);
        assert_eq!(cbuf.next_contiguous_slice_len(), CAPACITY - TRANSFER_SIZE);
    }

    #[test]
    #[should_panic]
    fn request_too_many_bytes_write() {
        const CAPACITY: usize = 8;
        let mut buf: [u8; CAPACITY] = [0; CAPACITY];
        let mut cbuf = super::Buffer::<u8>::new(&mut buf);
        const TRANSFER_SIZE: usize = 4;
        {
            let writable = cbuf.next_mut_slice(TRANSFER_SIZE);
            for i in 0..TRANSFER_SIZE {
                writable[i] = 1 + i as u8;
            }
        }

        assert_eq!(
            cbuf.next_contiguous_slice_len(),
            CAPACITY - TRANSFER_SIZE - 1
        );
        cbuf.next_mut_slice(TRANSFER_SIZE); // Not enough bytes to fulfill request; panic
    }

    #[test]
    #[should_panic]
    fn request_too_many_bytes_read() {
        const CAPACITY: usize = 8;
        let mut buf: [u8; CAPACITY] = [0; CAPACITY];
        let mut cbuf = super::Buffer::<u8>::new(&mut buf);
        const TRANSFER_SIZE: usize = 4;
        {
            let writable = cbuf.next_mut_slice(TRANSFER_SIZE);
            for i in 0..TRANSFER_SIZE {
                writable[i] = 1 + i as u8;
            }
        }

        let mut read_from: [u8; CAPACITY] = [0; CAPACITY];
        cbuf.take_slice(CAPACITY, &mut read_from); // Not enough bytes to fulfill request; panic
    }

    #[test]
    #[should_panic]
    fn request_peek_too_far() {
        const CAPACITY: usize = 8;
        let mut buf: [u8; CAPACITY] = [0; CAPACITY];
        let mut cbuf = super::Buffer::<u8>::new(&mut buf);
        const TRANSFER_SIZE: usize = 4;
        {
            let writable = cbuf.next_mut_slice(TRANSFER_SIZE);
            for i in 0..TRANSFER_SIZE {
                writable[i] = 1 + i as u8;
            }
        }

        cbuf.peek(TRANSFER_SIZE); // Peek index out of range
    }

    #[test]
    fn peek() {
        const CAPACITY: usize = 8;
        let mut buf: [u8; CAPACITY] = [0; CAPACITY];
        let mut cbuf = super::Buffer::<u8>::new(&mut buf);

        // Write 5 bytes (2 more available)
        {
            let writable = cbuf.next_mut_slice(5);
            for i in 0..5 {
                writable[i] = 1 + i as u8;
            }
        }
        assert_eq!(cbuf.size(), 5);

        // Read 3; throw away (3 + 2 available)
        let mut read_from: [u8; 3] = [0; 3];
        cbuf.take_slice(3, &mut read_from);
        assert_eq!(cbuf.size(), 2);
        assert_eq!(cbuf.peek(0), 4);
        assert_eq!(cbuf.peek(1), 5);

        // Fill up
        {
            {
                let len = cbuf.next_contiguous_slice_len();
                let writable = cbuf.next_mut_slice(len);
                for i in 0..len {
                    writable[i] = 6 + i as u8;
                }
            }
            {
                let len = cbuf.next_contiguous_slice_len();
                let writable = cbuf.next_mut_slice(len);
                for i in 0..len {
                    writable[i] = 9 + i as u8;
                }
            }
        }

        assert_eq!(cbuf.size(), CAPACITY - 1);
        for i in 0..CAPACITY - 1 {
            assert_eq!(4 + i as u8, cbuf.peek(i), "Index {}", i);
        }
    }
}
