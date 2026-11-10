# Sources and third-party notices

The monograph's bibliography is in `book/references.tex`; parameter-level provenance is in `book/evidence/current_parameter_ledger.json` and the accompanying profiles/manifests. No third-party reference book or research-paper PDF is redistributed.

| Material | Attribution and retained evidence |
|---|---|
| Bmatching | Henrik Reinstädtler, https://github.com/HeiHGM/Bmatching ; commit `2f456c32c1d28de28b02822d48e383d769e667ea`; MIT license retained under `book/reproduction/project/总结/07_证据与复现索引/超图建模与原生求解/Bmatching_author/` |
| Streaming hypergraph algorithm | Henrik Reinstädtler et al.; original MIT license retained in `Streaming_author/LICENSE` and `AUTHOR_LICENSE.txt`; source URL/identity in the original manifests |
| Abseil / Easylogging++ | Apache-2.0 / MIT; original LICENSE files retained alongside the bundled author dependencies |
| CMU energy-risk software | AirLab (2021), BSD 4-Clause. This product includes software developed by [project] (AirLab, `castacks/cvar-energy-risk-deep-model`). Original acknowledgment wording and complete license retained in the energy evidence archive |
| M100 measured data | Rodrigues et al., *Data Collected with Package Delivery Quadcopter Drone*, DOI [10.1184/R1/12683453.v1](https://doi.org/10.1184/R1/12683453.v1), CC BY 4.0. Original metadata, authors, README and downloaded data retained. Derived calibration results are separately identified; original data are not relabelled as M300 measurements |
| OpenStreetMap | © OpenStreetMap contributors, [ODbL](https://www.openstreetmap.org/copyright). Geographic JSON and source map data retain original attribution and acquisition records. Coordinates/roads are OSM-derived; inspection view templates and aircraft settings are research assumptions |
| Terrain and weather | Exact providers, acquisition requests, dates and reference fields remain in the scene/source manifests. Derived terrain and historical weather are not field survey or real-time forecast claims |
| Caballero reference scenario | Original source files and supplied LICENSE retained under geographic archive `sources/caballero2025_data/` |

Third-party licenses apply to their respective components; this repository does not relicense them. Project-original prose/code has no newly imposed blanket license. Third-party software omitted because its redistribution license was not established is listed, with file identities, in `data/external_assets.json`; its public source URLs and commits remain in the energy source manifest. Library dependencies are installed from their original packages rather than copied as an environment image.
