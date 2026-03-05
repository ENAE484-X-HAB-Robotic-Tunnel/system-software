{
    pkgs,
    inputs,
    ...
}:
inputs.treefmt-nix.lib.mkWrapper pkgs {
    projectRootFile = "flake.nix";

    # nix
    programs = {
        deadnix.enable = true;
        nixfmt = {
            enable = true;
            indent = 4;
        };
    };

    # md
    programs = {
        prettier = {
            enable = true;
            settings = {
                tabWidth = 4;
            };
        };
    };

    # c/cpp
    programs = {
        # clang
        # NOTE: unneeded, as none of our modules are written in cpp
        # clang-format.enable = true;
        # clang-tidy.enable = true;

        # cmake
        # TODO: contribute gersemi to treefmt-nix
        # gersemi.enable = true;
    };

    # python
    programs = {
        ruff-check.enable = true;
        ruff-format.enable = true;
    };
}
