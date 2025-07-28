import argparse, getpass
from ros_prompt.utilities.secrets import save_api_key, get_api_key

def main():
    p = argparse.ArgumentParser(prog="ros_prompt")
    sp = p.add_subparsers(dest="cmd", required=True)

    add = sp.add_parser("set-key", help="Store an API key securely")
    add.add_argument("provider", choices=["openai", "deepseek"])

    show = sp.add_parser("show-key", help="Print provider key location")

    args = p.parse_args()
    if args.cmd == "set-key":
        key = getpass.getpass(prompt="Paste secret key (will not echo): ")
        save_api_key(args.provider, key)
        print(f"{args.provider} key saved ✨")
    elif args.cmd == "show-key":
        for prov in ["openai", "deepseek"]:
            src = get_api_key(prov)
            print(f"{prov:7} → {'found' if src else 'not set'}")

if __name__ == "__main__":
    main()
