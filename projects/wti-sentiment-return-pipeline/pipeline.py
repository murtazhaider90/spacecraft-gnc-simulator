from __future__ import annotations

import argparse
import re
import numpy as np
import pandas as pd
from sklearn.linear_model import Ridge
from sklearn.metrics import mean_squared_error

POSITIVE = {"gain", "growth", "strong", "surge", "tight", "bullish", "rise"}
NEGATIVE = {"loss", "weak", "drop", "fall", "bearish", "slump", "risk"}
UNCERTAINTY = {"may", "might", "uncertain", "possible", "could", "risk"}
FORWARD = {"expect", "forecast", "outlook", "next", "future", "project"}


def text_features(text: str) -> np.ndarray:
    tokens = re.findall(r"[a-z]+", text.lower())
    n = max(1, len(tokens))
    return np.array([
        (sum(t in POSITIVE for t in tokens) - sum(t in NEGATIVE for t in tokens)) / n,
        sum(t in UNCERTAINTY for t in tokens) / n,
        sum(t in FORWARD for t in tokens) / n,
        np.log1p(len(tokens)),
    ])


def expanding_backtest(x: np.ndarray, y: np.ndarray, min_train: int = 40) -> tuple[np.ndarray, np.ndarray]:
    pred, truth = [], []
    for i in range(min_train, len(y)):
        model = Ridge(alpha=1.0)
        model.fit(x[:i], y[:i])
        pred.append(model.predict(x[i:i+1])[0])
        truth.append(y[i])
    return np.array(pred), np.array(truth)


def demo(seed: int = 7) -> None:
    rng = np.random.default_rng(seed)
    vocab = [
        "oil demand growth outlook strong", "inventory drop bullish market",
        "uncertain demand may weaken", "supply risk could rise",
        "bearish forecast possible fall", "production gain future tight market",
    ]
    texts = [vocab[i % len(vocab)] for i in range(180)]
    x = np.vstack([text_features(t) for t in texts])
    latent = 0.025 * x[:, 0] + 0.012 * x[:, 2]
    y = latent + rng.normal(0.0, 0.01, len(latent))
    pred, truth = expanding_backtest(x, y)
    rmse = mean_squared_error(truth, pred) ** 0.5
    direction = np.mean(np.sign(pred) == np.sign(truth))
    print(f"synthetic expanding-window RMSE: {rmse:.5f}")
    print(f"synthetic directional accuracy: {100*direction:.1f}%")
    print("Demo data are synthetic; these figures are not trading results.")


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--demo", action="store_true")
    args = p.parse_args()
    if args.demo:
        demo()
    else:
        p.error("provide --demo or integrate your own timestamped text/market data")


if __name__ == "__main__":
    main()
