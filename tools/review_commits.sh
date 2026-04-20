#!/bin/bash

# 检查指定范围的提交并使用 Claude Code 给出修改建议
# 用法: ./review_user_commits.sh <from_commit> <to_commit>
# 示例: ./review_user_commits.sh abc1234 def5678
#       ./review_user_commits.sh abc1234 HEAD

set -e

if [ -z "$1" ] || [ -z "$2" ]; then
    echo "用法: $0 <from_commit> <to_commit>"
    echo "示例: $0 abc1234 HEAD"
    echo "      $0 abc1234 def5678"
    exit 1
fi

FROM_COMMIT="$1"
TO_COMMIT="$2"
CURRENT_BRANCH=$(git branch --show-current)

echo "==================================="
echo "检查分支: $CURRENT_BRANCH"
echo "范围: $FROM_COMMIT .. $TO_COMMIT"
echo "==================================="
echo ""

# 验证 commit 是否存在
if ! git rev-parse --verify "$FROM_COMMIT" > /dev/null 2>&1; then
    echo "错误: 找不到 commit $FROM_COMMIT"
    exit 1
fi
if ! git rev-parse --verify "$TO_COMMIT" > /dev/null 2>&1; then
    echo "错误: 找不到 commit $TO_COMMIT"
    exit 1
fi

COUNT=$(git rev-list --count "$FROM_COMMIT^".."$TO_COMMIT")
if [ "$COUNT" -eq 0 ]; then
    echo "错误: 指定范围内没有提交"
    exit 1
fi

echo "找到 $COUNT 个提交"
echo ""

# 显示提交列表
echo "提交列表:"
echo "-----------------------------------"
git log --oneline "$FROM_COMMIT^".."$TO_COMMIT"
echo ""

# 获取 diff
echo "正在生成 diff..."
DIFF_CONTENT=$(git diff "$FROM_COMMIT^".."$TO_COMMIT")
DIFF_SIZE=$(echo "$DIFF_CONTENT" | wc -l)
echo "Diff 已生成 ($DIFF_SIZE 行)"
echo ""

echo "==================================="
echo "准备调用 Claude Code 进行代码审查..."
echo "==================================="
echo ""

COMMIT_LOG=$(git log "$FROM_COMMIT^".."$TO_COMMIT")

REVIEW_PROMPT="请审查以下提交的代码变更，并给出修改建议：

分支: $CURRENT_BRANCH
范围: $FROM_COMMIT .. $TO_COMMIT
提交数量: $COUNT

请重点关注：
1. 代码质量和可读性
2. 潜在的 bug 或逻辑错误
3. 性能问题
4. 代码规范和最佳实践
5. 边界条件处理
6. 是否有未处理的异常情况

提交信息和代码变更详见下方。

---

提交信息:
==========================================
$COMMIT_LOG

==========================================
代码变更:
==========================================
$DIFF_CONTENT
"

CLAUDE_BIN=$(ls /root/.vscode-server/extensions/anthropic.claude-code-*/resources/native-binary/claude 2>/dev/null | sort -V | tail -1)

if [ -z "$CLAUDE_BIN" ] || [ ! -f "$CLAUDE_BIN" ]; then
    echo "错误: 找不到 Claude Code CLI"
    exit 1
fi

SUGGESTION_DIR="$(pwd)/cc_suggestion"
mkdir -p "$SUGGESTION_DIR"
FROM_SHORT=$(echo "$FROM_COMMIT" | cut -c1-7)
TO_SHORT=$(echo "$TO_COMMIT" | cut -c1-7)
SUGGESTION_FILE="$SUGGESTION_DIR/review_${FROM_SHORT}_${TO_SHORT}_$(date +%Y%m%d_%H%M%S).md"

# 先写入基本信息和提交列表到报告文件
{
    echo "# 代码审查报告"
    echo ""
    echo "## 基本信息"
    echo ""
    echo "- **分支**: $CURRENT_BRANCH"
    echo "- **审查范围**: $FROM_COMMIT .. $TO_COMMIT"
    echo "- **提交数量**: $COUNT"
    echo "- **审查时间**: $(date '+%Y-%m-%d %H:%M:%S')"
    echo ""
    echo "## 提交列表"
    echo ""
    git log --format="- %h: %s" "$FROM_COMMIT^".."$TO_COMMIT"
    echo ""
    echo "---"
    echo ""
    echo "## Claude Code 审查建议"
    echo ""
} > "$SUGGESTION_FILE"

"$CLAUDE_BIN" -p --model sonnet "$REVIEW_PROMPT" | tee -a "$SUGGESTION_FILE"

echo ""
echo "==================================="
echo "审查完成"
echo "==================================="
echo ""
echo "建议已保存到: $SUGGESTION_FILE"
