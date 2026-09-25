# Kaucja.pl OpenAPI: what it is and how we use it

Source: "OpenAPI Kaucja.pl: szybki start dla sklepów" (quick start for shops), 11 pages, July 2025.
https://cdn.kaucja.pl/gcdeposits/media/2025OpenAPIKaucjaplszybkistartdlasklepwiproducentwRVM.pdf

## What the document says

It describes the **PoS Integration API**. It connects a shop's till (PoS) to the central system of the Kaucja.pl deposit operator (DRS). Everything happens in real time: container returns, vouchers, bag changes, collection-station registration.

| Flow | Calls in the diagrams | Meaning |
|---|---|---|
| 1. Return of containers | `POST /transaction` | The till accepts or rejects each container **locally**, then sends one transaction at the end: EANs, time, place, collection-station number. The reply is OK (cash payout) or voucher details |
| 2. Vouchers | `GET /voucher/{voucherId}`, `POST /voucher` | Check a voucher's status (valid, used, expired). Report it as redeemed as a discount or as cash |
| 3. Bag change and sealing | `/bag-replacement` | A full bag gets a seal with a unique code. The till registers the seal number and the theoretical weight, computed from the EANs |
| 3b. Seal swap | `/swap-seal` | Damaged bag: old seal and new seal numbers |
| 4. Collection stations | (registration, path not given) | Each station/till/machine gets a unique **ID** from the DRS, and that ID is used in every later call |

## What the document does NOT say

- No base URL, no authentication, no request/response schemas, no sandbox.
- **No "is this EAN a deposit product?" lookup.** In the diagram the till decides "accept or reject bottle" on its own, so the product list lives on the shop side.
- Access is for registered shops and machine (RVM) makers. We are neither, so assume **no real access during the hackathon** unless the organizers can arrange a sandbox.

## How we use it

1. **Our station behaves like a small RVM** and follows the same flow: scan EAN → accept or reject locally → at the end, one `POST /transaction` → voucher.
2. **`station/drs.py`** is a client with exactly these calls: `transaction`, `get_voucher`, `redeem_voucher`, `bag_replacement`. It points at **our own mock server** (`tools/mock_drs.py`) by default. If we get real credentials, we change the URL and the auth header in `demo.yaml`.
3. **Deposit check = local EAN list** (`config/deposit_eans.json`), like a real till. Before the event, scan all demo bottles and cans and enter them with a type and deposit amount. Add one or two non-deposit items (a glass jar, a juice carton) to show a reject.
4. **Voucher:** the mock returns a voucher ID + amount; the dashboard shows it as a QR code and a big number. **Bag full** (N containers) → `bag_replacement` with a printed seal code, as a Tier 3 beat.

## Say it honestly in the pitch

"We built our station against the Kaucja.pl OpenAPI flow: transaction, voucher, bag replacement. Today it talks to a mock with the same endpoints, because the API is only open to registered shops." Say "connected to the real system" only if we actually get access.

## Ask the organizers (and Kaucja.pl if there is a contact)

- Is there a sandbox or test key for the PoS Integration API?
- Is there an official list of registered deposit EANs (a producer registry or open dataset)?
