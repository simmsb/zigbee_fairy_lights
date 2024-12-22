{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    # esp.url = "github:mirrexagon/nixpkgs-esp-dev";
    esp.url = "github:quentinmit/nixpkgs-esp-dev?ref=tools";
    esp.inputs.nixpkgs.follows = "nixpkgs";

    parts.url = "github:hercules-ci/flake-parts";
    parts.inputs.nixpkgs-lib.follows = "nixpkgs";
  };

  outputs = inputs@{ self
    , nixpkgs
    , parts
    , ...
    }:
    parts.lib.mkFlake { inherit inputs; } {
      systems = nixpkgs.lib.systems.flakeExposed;
      imports = [
      ];
      perSystem = { config, pkgs, system, lib, ... }:
      {
        devShells.default = pkgs.mkShell {
          name = "esp-idf-esp32c6-shell";

          buildInputs = with pkgs; [
            inputs.esp.packages.${system}.esp-idf-esp32c6
          ];

          IDF_PACKAGE = inputs.esp.packages.${system}.esp-idf-esp32c6;
        };
      };
  };
}
