#!/usr/bin/env python3
"""Quick OpenAI API key validation script.

Usage:
  export OPENAI_API_KEY="sk-..."
  python3 scripts/check_openai_api_key.py

Optional:
  python3 scripts/check_openai_api_key.py --api-key sk-... --model gpt-4.1-mini
  python3 scripts/check_openai_api_key.py --base-url https://your-proxy.example/v1
"""

from __future__ import annotations

import argparse
import os
import sys


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate an OpenAI API key.")
    parser.add_argument(
        "--api-key",
        default=os.environ.get("OPENAI_API_KEY"),
        help="OpenAI API key. Defaults to OPENAI_API_KEY.",
    )
    parser.add_argument(
        "--base-url",
        default=os.environ.get("OPENAI_BASE_URL"),
        help="Optional custom base URL / proxy endpoint.",
    )
    parser.add_argument(
        "--model",
        default=None,
        help="Optional model name. If provided, also performs a tiny Responses API call.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if not args.api_key:
        print("ERROR: missing API key. Set OPENAI_API_KEY or pass --api-key.", file=sys.stderr)
        return 2

    try:
        from openai import (
            APIConnectionError,
            APIStatusError,
            AuthenticationError,
            NotFoundError,
            OpenAI,
            PermissionDeniedError,
            RateLimitError,
        )
    except ImportError:
        print(
            "ERROR: openai package is not installed.\n"
            "Install it with: pip install openai",
            file=sys.stderr,
        )
        return 2

    client_kwargs = {"api_key": args.api_key}
    if args.base_url:
        client_kwargs["base_url"] = args.base_url

    try:
        client = OpenAI(**client_kwargs)

        # Low-cost auth check: list models accessible to this key.
        models_page = client.models.list()
        model_ids = sorted(model.id for model in models_page.data)

        print("OK: API key is valid and the API is reachable.")
        print(f"Accessible models: {len(model_ids)}")
        preview = model_ids[:10]
        if preview:
            print("First models:")
            for model_id in preview:
                print(f"  - {model_id}")

        if args.model:
            print(f"\nTesting a minimal Responses API call with model: {args.model}")
            response = client.responses.create(
                model=args.model,
                input="Reply with exactly: OK",
                max_output_tokens=8,
            )

            text = getattr(response, "output_text", "") or ""
            print("Responses API call succeeded.")
            print(f"Model reply: {text.strip() or '<empty>'}")

        return 0

    except AuthenticationError as exc:
        print(f"AUTH ERROR: {exc}", file=sys.stderr)
        return 1
    except PermissionDeniedError as exc:
        print(f"PERMISSION ERROR: {exc}", file=sys.stderr)
        return 1
    except NotFoundError as exc:
        print(f"NOT FOUND: {exc}", file=sys.stderr)
        return 1
    except RateLimitError as exc:
        print(f"RATE LIMIT / BILLING ISSUE: {exc}", file=sys.stderr)
        return 1
    except APIConnectionError as exc:
        print(f"NETWORK ERROR: {exc}", file=sys.stderr)
        return 1
    except APIStatusError as exc:
        print(f"API STATUS ERROR: status={exc.status_code} message={exc}", file=sys.stderr)
        return 1
    except Exception as exc:
        print(f"UNEXPECTED ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
