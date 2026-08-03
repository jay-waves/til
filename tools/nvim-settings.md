### nvim on windows 

配置放在 `~/AppData/Local/nvim/init.lua` 

从 nvim 0.12.0 开始，直接用内置包管理器即可。插件不用太多：
* 主题: github-nvim-theme 
* 状态栏、Tab 栏: lualine.nvim 
* 语法解析（高亮）：nvim-treesitter
* GIT 状态、差异对比： gitsigns 
* 补全：blink.cmp 
* find, grep 集成：fzf-lua 

不太喜欢 nvim 内部的文件树插件，我主要用：
* `:e .` 内置的文件选择器 
* [`lf`](https://github.com/gokcehan/lf) 命令行文件管理器，GO 写的，很轻量好用。

```lua
vim.opt.number = true
vim.opt.tabstop = 4
vim.opt.shiftwidth = 4
vim.opt.cursorline = true
vim.opt.smartindent = true
vim.opt.showcmd = true
vim.opt.wrap = false
vim.opt.ignorecase = true
vim.opt.smartcase = true
vim.opt.incsearch = true
vim.opt.hlsearch = true
vim.opt.termguicolors = true
vim.opt.laststatus = 2

vim.keymap.set("n", "j", "gj", { noremap = true })
vim.keymap.set("n", "k", "gk", { noremap = true })

vim.pack.add({
  "https://github.com/projekt0n/github-nvim-theme",
  "https://github.com/nvim-lualine/lualine.nvim",
  "https://github.com/nvim-treesitter/nvim-treesitter",
  "https://github.com/lewis6991/gitsigns.nvim",
  "https://github.com/ibhagwan/fzf-lua",

  { src = "https://github.com/saghen/blink.cmp", version = "v1", },
})


require("github-theme").setup({ options = { transparent = true, }, })

vim.cmd.colorscheme("github_dark_tritanopia")

require("lualine").setup({
  options = {
    theme = "auto", 
    icons_enabled = true,
    globalstatus = true,
  },

  sections = {
    lualine_a = { "mode" },
    lualine_c = { { "filename", path = 1, } },
    lualine_x = { "filetype" },
    lualine_y = { "progress" },
    -- lualine_z = { "location" },
  },

  tabline = {
    lualine_a = {
      {
        "buffers",
        mode = 0, 
        show_filename_only = true,
        show_modified_status = true,
		max_length = function()
			return math.floor(vim.o.columns * 0.9)
		end,
      }
    },
  },
})

require("gitsigns").setup({})
vim.api.nvim_create_user_command("GitPreview",
	function()
		require("gitsigns").preview_hunk()
	end,
{})
vim.api.nvim_create_user_command("GitNext",
  function()
    require("gitsigns").next_hunk()
  end,
{})
vim.api.nvim_create_user_command("GitPrev",
  function()
    require("gitsigns").prev_hunk()
  end,
{})
vim.api.nvim_create_user_command("GitBlame",
  function()
    require("gitsigns").blame_line()
  end,
{})

--> TreeSitter
require("nvim-treesitter").install({
  "lua",
  "vim",
  "vimdoc",
  "query",
  "markdown",
  "markdown_inline",
  "bash",
  "python",
  "javascript",
  "typescript",
  "tsx",
  "html",
  "xml",
  "css",
  "json",
  "yaml",
  "go",
  "rust",
  "c",
  "cpp",
  "typst"
})

vim.api.nvim_create_autocmd("FileType", {
  callback = function()
    pcall(vim.treesitter.start)
  end,
})

--> FzfLua 
vim.keymap.set("n", "gp", "<cmd>FzfLua<cr>", {
  desc = "Command Palette",
})


--> Blink.CMP
require("blink.cmp").setup({
  keymap = {
    preset = "super-tab",
  },

  sources = {
    default = {
      "lsp",
      "path",
      "snippets",
      "buffer",
    },
  },

  completion = {
    menu = {
      auto_show = true,
    },

    documentation = {
      auto_show = false,
    },

    ghost_text = {
      enabled = true,
      show_with_menu = true,
    },
  },
})

```

### lsp 

lsp code jump:
* `grt` type definition 
* `gri` implementation 
* `grr` references
* `gra` code actions 
* `grn` rename 
* `K`   hover 
* `Ctrl-]` definition (gvim)
