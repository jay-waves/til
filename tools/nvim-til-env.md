
## 查看报错 

neovim 原生的查看 lsp 报错的命令有些复杂，包装一个自定义命令：

```lua 
vim.api.nvim_create_user_command("Diag", function()
  vim.diagnostic.open_float({
    scope = "line",
    source = true,
  })
end, {
  desc = "show diagnostic of current line",
})

vim.api.nvim_create_user_command("DiagList", function()
  vim.diagnostic.setloclist()
  vim.cmd.lopen()
end, {
  desc = "show diagnostic of this buffer",
})

```

## markdown 环境

使用 [`selimacerbas/markdown-preview.nvim`](https://github.com/selimacerbas/markdown-preview.nvim) 插件最方便。

> 相比上游，该插件是纯前端，所有 JS 组件都是从 CDN 现场拉取的，没有独立的 NodeJS 运行时。
> 该插件唯一的缺点：点击 Previewer 位置，不能反向同步到 Neovim。我自己 fork 修改了这个

我个人的习惯是，只开启一个 Previewer Server，然后切换 Buffer 时，Previewer 同步切换过去。这需要：
* Previewer 生命周期类似 LSP，即和 Neovim 实例绑定
* Previewer 支持随机端口，避免多个 Neovim 实例导致 Previewer 撞端口。

```lua 
vim.pack.add({
  "https://github.com/jay-waves/markdown-preview.nvim",
})

--> 我个人使用的 theme.css, highlight.css, 需要放在 nvim-data 目录下
local markdown_css_dir = vim.fs.joinpath(vim.fn.stdpath("config"), "css")
local markdown_css = {
  vim.fs.joinpath(markdown_css_dir, "theme.css"),
  vim.fs.joinpath(markdown_css_dir, "highlight.css"),
}

require("markdown_preview").setup({
  custom_css = markdown_css,
  default_theme = "auto",
  follow_current_buffer = true,
})

```

## typst 环境

Typst 有一个很强的 lsp： [Tinymist](https://github.com/Myriad-Dreamin/tinymist)，内置 Previewer 和热更新能力。

由于 Typst 只能在文件系统 Sandbox 内编译，需要告知 Tinymist 项目根目录。Neovim 中可以复用 `.git` 的位置。

```lua
vim.api.nvim_create_autocmd("FileType", {
	pattern = "typst",
	callback = function()
		vim.opt_local.backupcopy = "yes"
	end,
})

vim.api.nvim_create_user_command("TypstPreview", function()
  local bufnr = vim.api.nvim_get_current_buf()
  local client = vim.lsp.get_clients({
    bufnr = bufnr,
    name = "tinymist",
  })[1]

  if not client then
    vim.notify(
      "Tinymist is not attached to the current buffer",
      vim.log.levels.ERROR
    )
    return
  end

  client:exec_cmd({
    title = "Start Tinymist Preview",
    command = "tinymist.startDefaultPreview",
    arguments = {},
  }, {
    bufnr = bufnr,
  }, function(err)
    if err then
      vim.notify(
        vim.inspect(err),
        vim.log.levels.ERROR,
        { title = "Tinymist Preview" }
      )
    end
  end)

end, {
  desc = "start Tinymist preview for the focused Typst buffer",
})

vim.api.nvim_create_autocmd("BufEnter", {
  pattern = "*.typ",
  callback = function(args)
    vim.schedule(function()
      if vim.api.nvim_get_current_buf() ~= args.buf then
        return
      end

      local client = vim.lsp.get_clients({
        bufnr = args.buf,
        name = "tinymist",
      })[1]

      if not client then
        return
      end

      client:exec_cmd({
        title = "Focus Tinymist Preview",
        command = "tinymist.focusMain",
        arguments = { vim.api.nvim_buf_get_name(args.buf) },
      }, {
        bufnr = args.buf,
      })
    end)
  end,
  desc = "focus Tinymist preview when entering a Typst buffer",
})

vim.lsp.config("tinymist", {
  cmd = { "tinymist", "lsp" },
  filetypes = { "typst" },

  capabilities = require("blink.cmp").get_lsp_capabilities(),

  root_markers = {
	  ".typst",
	  ".git",
  },

  settings = {
    rootPath = ".",
	projectResolution = "singleFile",
    formatterMode = "typstyle",
  },
})

vim.lsp.enable("tinymist")

```
