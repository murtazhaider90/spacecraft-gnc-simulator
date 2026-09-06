# WTI Crude-Oil Sentiment & Return-Prediction Pipeline

Research pipeline for testing whether text-derived sentiment features contain predictive information for subsequent WTI crude-oil returns.

## Design

- Ingest timestamped article/headline text and market observations.
- Extract transparent lexicon-based polarity, uncertainty and forward-looking features.
- Align text features to later return windows without look-ahead leakage.
- Train a regularized linear baseline.
- Evaluate strictly out of sample using expanding-window splits.
- Report prediction error, directional accuracy and coefficient stability.

The repository ships a deterministic synthetic demo so the full research workflow is runnable without redistributing licensed news or market data. Any real-data study should document data provenance, timestamp conventions, transaction costs and the distinction between statistical predictability and tradability.

```bash
pip install numpy pandas scikit-learn
python pipeline.py --demo
```
