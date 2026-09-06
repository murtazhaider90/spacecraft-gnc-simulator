from __future__ import annotations

from dataclasses import dataclass
import random


@dataclass
class TopOfBook:
    best_bid: int = 0
    best_ask: int = 0xFFFFFFFF

    def update(self, is_bid: bool, price: int) -> None:
        if not 0 <= price <= 0xFFFFFFFF:
            raise ValueError("price must fit uint32")
        if is_bid:
            self.best_bid = max(self.best_bid, price)
        else:
            self.best_ask = min(self.best_ask, price)

    @property
    def spread(self) -> int:
        return max(0, self.best_ask - self.best_bid)

    @property
    def mid(self) -> int:
        return (self.best_bid + self.best_ask) >> 1


def randomized_demo(seed: int = 7, n: int = 1000) -> None:
    rng = random.Random(seed)
    book = TopOfBook()
    for _ in range(n):
        is_bid = bool(rng.getrandbits(1))
        price = rng.randint(90_00, 110_00)
        book.update(is_bid, price)
    print(book)
    print("spread ticks:", book.spread)
    print("mid ticks:", book.mid)


if __name__ == "__main__":
    randomized_demo()
