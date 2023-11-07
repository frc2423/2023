<script>
  let count = 0
  const increment = () => {
    count += 3030303;
  }
</script>

<style>
  button {
    background: indigo;
    font-size: 30px;
  }
</style>

<button on:click={increment}>
  count is {count}
</button>
